#include "plugin.h"
#include "CTimer.h"
#include "CVehicle.h"
#include "RenderWare.h"
#include "common.h"

using namespace plugin;

float g_beltOffset = 0.0f;
const float BELT_SPEED = 0.45f;   // adjust this value for speed

void OffsetGeometryUVs(RpGeometry* geometry, float offsetX)
{
    if (!geometry)
        return;

    // Most reliable way across plugin-sdk versions
    RwTexCoords* texCoords = geometry->texCoords[0];
    if (!texCoords)
        return;

    const RwInt32 numVerts = geometry->numVertices;

    RpGeometryLock(geometry, rpGEOMETRYLOCKTEXCOORDS);

    for (RwInt32 i = 0; i < numVerts; ++i)
    {
        texCoords[i].u += offsetX;
    }

    RpGeometryUnlock(geometry);
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

                const float delta = CTimer::ms_fTimeStep * BELT_SPEED;

                g_beltOffset += delta;
                if (g_beltOffset > 1000.0f)
                    g_beltOffset -= 1000.0f;

                RpClumpForAllAtomics(vehicle->m_pRwClump,
                    [](RpAtomic* atomic, void* data) -> RpAtomic*
                    {
                        float delta = *static_cast<float*>(data);

                        if (!atomic || !atomic->geometry)
                            return atomic;

                        bool hasBelt = false;

                        RpGeometryForAllMaterials(atomic->geometry,
                            [](RpMaterial* mat, void* data) -> RpMaterial*
                            {
                                bool* found = static_cast<bool*>(data);
                                if (mat && mat->texture && _stricmp(mat->texture->name, "f_belt") == 0)
                                {
                                    *found = true;
                                }
                                return mat;
                            },
                            &hasBelt);

                        if (hasBelt)
                        {
                            OffsetGeometryUVs(atomic->geometry, delta);
                        }

                        return atomic;
                    },
                    const_cast<float*>(&delta));   // pass delta safely
            };
    }
} fBeltScroller;
