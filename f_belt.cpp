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
const float GAS_BOOST = 0.020f;

struct OriginalUVs
{
    std::vector<RwTexCoords> uvs;
};

static std::unordered_map<RpGeometry*, OriginalUVs> g_originalUVs;
static std::unordered_map<int, float> g_constantOffsets;
static std::unordered_map<int, float> g_gasOffsets;
static std::unordered_map<int, int>   g_modelRefCount;
static std::unordered_set<int>        g_advancedThisFrame;
static unsigned int g_lastFrameCounter = 0;

void ApplyScroll(RpGeometry* geometry, float offset)
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

float GetMultiplier(const char* name)
{
    if (!name) return 1.0f;
    const char* mu = strstr(name, "_mu=");
    if (!mu) return 1.0f;
    return static_cast<float>(atof(mu + 4));
}

bool IsBelt(const char* name) { return name && _strnicmp(name, "f_belt", 6) == 0; }
bool IsChain(const char* name) { return name && _strnicmp(name, "f_chain", 7) == 0; }

const char* FindBeltOrChain(RwFrame* frame)
{
    while (frame)
    {
        char* name = GetFrameNodeName(frame);
        if (name && (IsBelt(name) || IsChain(name)))
            return name;
        frame = RwFrameGetParent(frame);
    }
    return nullptr;
}

class BeltAndChain
{
public:
    BeltAndChain()
    {
        Events::vehicleSetModelEvent += [](CVehicle* v, int modelId) {
            g_modelRefCount[modelId]++;
            };

        Events::vehicleDtorEvent.before += [](CVehicle* v) {
            int modelId = v->m_nModelIndex;
            auto it = g_modelRefCount.find(modelId);
            if (it != g_modelRefCount.end()) {
                if (--it->second <= 0) {
                    g_modelRefCount.erase(it);
                    g_constantOffsets.erase(modelId);
                    g_gasOffsets.erase(modelId);
                }
            }
            };

        Events::vehicleRenderEvent.before += [](CVehicle* vehicle) {
            if (!vehicle || !vehicle->m_pRwClump || !vehicle->bEngineOn)
                return;

            const int modelId = vehicle->m_nModelIndex;

            unsigned int frame = CTimer::m_FrameCounter;
            if (frame != g_lastFrameCounter) {
                g_advancedThisFrame.clear();
                g_lastFrameCounter = frame;
            }

            if (g_advancedThisFrame.find(modelId) == g_advancedThisFrame.end()) {
                float dt = CTimer::ms_fTimeStep;
                g_constantOffsets[modelId] += dt * DEFAULT_BELT_SPEED;
                float gas = fabsf(vehicle->m_fGasPedal);
                g_gasOffsets[modelId] += dt * (DEFAULT_BELT_SPEED + gas * GAS_BOOST);
                g_advancedThisFrame.insert(modelId);
            }

            float constantOffset = g_constantOffsets[modelId];
            float gasOffset = g_gasOffsets[modelId];

            static std::unordered_set<RpGeometry*> processed;
            processed.clear();

            RpClumpForAllAtomics(vehicle->m_pRwClump, [](RpAtomic* atomic, void* data) -> RpAtomic* {
                auto* offsets = static_cast<std::pair<float, float>*>(data);
                if (!atomic || !atomic->geometry || processed.count(atomic->geometry))
                    return atomic;

                const char* name = FindBeltOrChain(RpAtomicGetFrame(atomic));
                if (!name) return atomic;

                float mult = GetMultiplier(name);
                float offset = IsChain(name) ? offsets->second * mult : offsets->first * mult;

                ApplyScroll(atomic->geometry, offset);
                processed.insert(atomic->geometry);
                return atomic;
                }, new std::pair<float, float>(constantOffset, gasOffset));
            };
    }
} beltAndChain;
