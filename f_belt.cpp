#include "plugin.h"
#include "CTimer.h"
#include "CVehicle.h"
#include "CAutomobile.h"
#include "RenderWare.h"
#include "common.h"
#include "NodeName.h"
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <cstring>
#include <cstdlib>
#include <cmath>

using namespace plugin;

// ===================== TUNABLE =====================
const float DEFAULT_BELT_SPEED = 0.012f;   // f_belt
const float GAS_BOOST = 0.020f;   // f_chain extra at full gas
const float TRACK_SCALE = 0.15f;    // track scroll vs wheel rotation
// ===================================================

struct OriginalUVs
{
    std::vector<RwTexCoords> uvs;
};

struct BeltRenderData
{
    float constantOffset;
    float gasOffset;
    float trackOffset[4];   // 0=LF, 1=LB, 2=RF, 3=RB
    bool  isAutomobile;
};

static std::unordered_map<RpGeometry*, OriginalUVs> g_originalUVs;
static std::unordered_map<int, float> g_constantOffsets;
static std::unordered_map<int, float> g_gasOffsets;
static std::unordered_map<int, int>   g_modelRefCount;
static std::unordered_set<int>        g_advancedThisFrame;
static unsigned int g_lastFrameCounter = 0;

void ApplyBeltScroll(RpGeometry* geometry, float offset)
{
    if (!geometry || !geometry->texCoords[0])
        return;

    if (g_originalUVs.find(geometry) == g_originalUVs.end())
    {
        OriginalUVs data;
        data.uvs.resize(geometry->numVertices);
        RwTexCoords* src = geometry->texCoords[0];
        for (RwInt32 i = 0; i < geometry->numVertices; ++i)
            data.uvs[i] = src[i];
        g_originalUVs[geometry] = std::move(data);
    }

    const auto& original = g_originalUVs[geometry].uvs;
    RwTexCoords* dst = geometry->texCoords[0];
    float wrapped = offset - floorf(offset);

    RpGeometryLock(geometry, rpGEOMETRYLOCKTEXCOORDS);
    for (RwInt32 i = 0; i < geometry->numVertices; ++i)
    {
        dst[i].u = original[i].u + wrapped;
        dst[i].v = original[i].v;
    }
    RpGeometryUnlock(geometry);
}

float GetSpeedMultiplier(const char* name)
{
    if (!name) return 1.0f;
    const char* mu = strstr(name, "_mu=");
    if (!mu) return 1.0f;
    return static_cast<float>(atof(mu + 4));
}

// Returns wheel index or -1
// 0 = LF, 1 = LB, 2 = RF, 3 = RB
int GetTrackWheel(const char* name)
{
    if (!name) return -1;

    if (_strnicmp(name, "f_track_lf", 10) == 0) return 0;
    if (_strnicmp(name, "f_track_lb", 10) == 0) return 1;
    if (_strnicmp(name, "f_track_rf", 10) == 0) return 2;
    if (_strnicmp(name, "f_track_rb", 10) == 0) return 3;

    return -1;
}

bool IsBelt(const char* name)
{
    return name && _strnicmp(name, "f_belt", 6) == 0;
}

bool IsChain(const char* name)
{
    return name && _strnicmp(name, "f_chain", 7) == 0;
}

const char* FindSpecialNodeName(RwFrame* frame)
{
    while (frame)
    {
        char* name = GetFrameNodeName(frame);
        if (name)
        {
            if (_strnicmp(name, "f_belt", 6) == 0 ||
                _strnicmp(name, "f_chain", 7) == 0 ||
                _strnicmp(name, "f_track", 7) == 0)
            {
                return name;
            }
        }
        frame = RwFrameGetParent(frame);
    }
    return nullptr;
}

class FBeltScroller
{
public:
    FBeltScroller()
    {
        Events::vehicleSetModelEvent += [](CVehicle* vehicle, int modelId)
            {
                g_modelRefCount[modelId]++;
            };

        Events::vehicleDtorEvent.before += [](CVehicle* vehicle)
            {
                int modelId = vehicle->m_nModelIndex;
                auto it = g_modelRefCount.find(modelId);
                if (it != g_modelRefCount.end())
                {
                    it->second--;
                    if (it->second <= 0)
                    {
                        g_modelRefCount.erase(it);
                        g_constantOffsets.erase(modelId);
                        g_gasOffsets.erase(modelId);
                    }
                }
            };

        Events::vehicleRenderEvent.before += [](CVehicle* vehicle)
            {
                if (!vehicle || !vehicle->m_pRwClump)
                    return;

                const int modelId = vehicle->m_nModelIndex;
                bool engineOn = vehicle->bEngineOn;

                unsigned int currentFrame = CTimer::m_FrameCounter;
                if (currentFrame != g_lastFrameCounter)
                {
                    g_advancedThisFrame.clear();
                    g_lastFrameCounter = currentFrame;
                }

                if (engineOn && g_advancedThisFrame.find(modelId) == g_advancedThisFrame.end())
                {
                    float dt = CTimer::ms_fTimeStep;
                    g_constantOffsets[modelId] += dt * DEFAULT_BELT_SPEED;

                    float gas = fabsf(vehicle->m_fGasPedal);
                    float gasSpeed = DEFAULT_BELT_SPEED + gas * GAS_BOOST;
                    g_gasOffsets[modelId] += dt * gasSpeed;

                    g_advancedThisFrame.insert(modelId);
                }

                BeltRenderData rd = {};
                rd.constantOffset = g_constantOffsets[modelId];
                rd.gasOffset = g_gasOffsets[modelId];
                rd.isAutomobile = false;

                if (vehicle->m_nVehicleClass == VEHICLE_AUTOMOBILE ||
                    vehicle->m_nVehicleClass == VEHICLE_MTRUCK ||
                    vehicle->m_nVehicleClass == VEHICLE_QUAD)
                {
                    CAutomobile* autoMobile = static_cast<CAutomobile*>(vehicle);
                    rd.isAutomobile = true;

                    // 0=LF, 1=LB, 2=RF, 3=RB
                    rd.trackOffset[0] = autoMobile->m_fWheelRotation[0] * TRACK_SCALE;
                    rd.trackOffset[1] = autoMobile->m_fWheelRotation[1] * TRACK_SCALE;
                    rd.trackOffset[2] = autoMobile->m_fWheelRotation[2] * TRACK_SCALE;
                    rd.trackOffset[3] = autoMobile->m_fWheelRotation[3] * TRACK_SCALE;
                }

                static std::unordered_set<RpGeometry*> processed;
                processed.clear();

                RpClumpForAllAtomics(vehicle->m_pRwClump,
                    [](RpAtomic* atomic, void* data) -> RpAtomic*
                    {
                        BeltRenderData* rd = static_cast<BeltRenderData*>(data);

                        if (!atomic || !atomic->geometry)
                            return atomic;

                        if (processed.count(atomic->geometry))
                            return atomic;

                        const char* name = FindSpecialNodeName(RpAtomicGetFrame(atomic));
                        if (!name)
                            return atomic;

                        float multiplier = GetSpeedMultiplier(name);
                        float finalOffset = 0.0f;
                        bool apply = false;

                        if (IsBelt(name))
                        {
                            finalOffset = rd->constantOffset * multiplier;
                            apply = true;
                        }
                        else if (IsChain(name))
                        {
                            finalOffset = rd->gasOffset * multiplier;
                            apply = true;
                        }
                        else
                        {
                            int wheel = GetTrackWheel(name);
                            if (wheel >= 0 && rd->isAutomobile)
                            {
                                finalOffset = rd->trackOffset[wheel] * multiplier;
                                apply = true;
                            }
                        }

                        if (apply)
                        {
                            ApplyBeltScroll(atomic->geometry, finalOffset);
                            processed.insert(atomic->geometry);
                        }

                        return atomic;
                    },
                    &rd);
            };
    }
} fBeltScroller;
