#include "plugin.h"
#include "CTimer.h"
#include "CVehicle.h"
#include "RenderWare.h"
#include "common.h"
#include "NodeName.h"
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <cstring>
#include <cstdlib>

using namespace plugin;

const float DEFAULT_BELT_SPEED = 0.012f;
const float GAS_BOOST = 0.020f;   // how much faster it gets at full gas

struct OriginalUVs
{
    std::vector<RwTexCoords> uvs;
};

struct BeltRenderData
{
    float constantOffset;
    float gasOffset;
};

static std::unordered_map<RpGeometry*, OriginalUVs> g_originalUVs;

// Two separate running offsets per model
static std::unordered_map<int, float> g_constantOffsets;
static std::unordered_map<int, float> g_gasOffsets;

static std::unordered_map<int, int> g_modelRefCount;
static std::unordered_set<int> g_advancedThisFrame;
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

float GetBeltSpeedMultiplier(const char* name)
{
    if (!name) return 1.0f;
    const char* mu = strstr(name, "_mu=");
    if (!mu) return 1.0f;
    return static_cast<float>(atof(mu + 4));
}

bool IsGasBelt(const char* name)
{
    return name && _strnicmp(name, "f_beltg", 7) == 0;
}

const char* FindBeltNodeName(RwFrame* frame)
{
    while (frame)
    {
        char* name = GetFrameNodeName(frame);
        if (name && _strnicmp(name, "f_belt", 6) == 0)
            return name;
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

                if (!vehicle->bEngineOn)
                    return;

                const int modelId = vehicle->m_nModelIndex;

                unsigned int currentFrame = CTimer::m_FrameCounter;
                if (currentFrame != g_lastFrameCounter)
                {
                    g_advancedThisFrame.clear();
                    g_lastFrameCounter = currentFrame;
                }

                // Advance the two offsets only once per model per frame
                if (g_advancedThisFrame.find(modelId) == g_advancedThisFrame.end())
                {
                    float dt = CTimer::ms_fTimeStep;

                    // Constant belts always run at base speed
                    g_constantOffsets[modelId] += dt * DEFAULT_BELT_SPEED;

                    // Gas belts run at base speed + extra from gas pedal
                    float gas = fabsf(vehicle->m_fGasPedal);          // 0.0 → 1.0
                    float gasSpeed = DEFAULT_BELT_SPEED + gas * GAS_BOOST;
                    g_gasOffsets[modelId] += dt * gasSpeed;

                    g_advancedThisFrame.insert(modelId);
                }

                BeltRenderData rd;
                rd.constantOffset = g_constantOffsets[modelId];
                rd.gasOffset = g_gasOffsets[modelId];

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

                        const char* name = FindBeltNodeName(RpAtomicGetFrame(atomic));
                        if (!name)
                            return atomic;

                        float multiplier = GetBeltSpeedMultiplier(name);

                        float finalOffset;
                        if (IsGasBelt(name))
                            finalOffset = rd->gasOffset * multiplier;
                        else
                            finalOffset = rd->constantOffset * multiplier;

                        ApplyBeltScroll(atomic->geometry, finalOffset);
                        processed.insert(atomic->geometry);
                        return atomic;
                    },
                    &rd);
            };
    }
} fBeltScroller;
