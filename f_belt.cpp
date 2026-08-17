#include "plugin.h"
#include "CTimer.h"
#include "CVehicle.h"
#include "RenderWare.h"
#include "common.h"
#include "NodeName.h"          // ← this is the key
#include <unordered_map>
#include <vector>
#include <cstring>
#include <cstdlib>

using namespace plugin;

const float DEFAULT_BELT_SPEED = 0.012f;

struct OriginalUVs
{
    std::vector<RwTexCoords> uvs;
};

static std::unordered_map<RpGeometry*, OriginalUVs> g_originalUVs;
static float g_beltOffset = 0.0f;

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

// Walk up the parent chain looking for a frame that starts with "f_belt"
const char* FindBeltNodeName(RwFrame* frame)
{
    while (frame)
    {
        char* name = GetFrameNodeName(frame);   // from NodeName.h
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
        Events::vehicleRenderEvent.before += [](CVehicle* vehicle)
            {
                if (!vehicle || !vehicle->m_pRwClump)
                    return;

                g_beltOffset += CTimer::ms_fTimeStep * DEFAULT_BELT_SPEED;

                RpClumpForAllAtomics(vehicle->m_pRwClump,
                    [](RpAtomic* atomic, void*) -> RpAtomic*
                    {
                        if (!atomic || !atomic->geometry)
                            return atomic;

                        const char* name = FindBeltNodeName(RpAtomicGetFrame(atomic));
                        if (!name)
                            return atomic;

                        float multiplier = GetBeltSpeedMultiplier(name);
                        ApplyBeltScroll(atomic->geometry, g_beltOffset * multiplier);

                        return atomic;
                    }, nullptr);
            };
    }
} fBeltScroller;
