#include "plugin.h"
#include "CVehicle.h"
#include "CAutomobile.h"
#include "RenderWare.h"
#include "NodeName.h"
#include <unordered_map>
#include <cstring>

using namespace plugin;

const float ROTATION_PER_UNIT = 80.0f;   // degrees per unit of Z travel

struct SuspOriginalMatrix
{
    RwMatrix mat;
    bool stored = false;
};

struct WheelRestZ
{
    float z[4] = {};
    bool initialized = false;
};

static std::unordered_map<RwFrame*, SuspOriginalMatrix> g_suspMatrices;
static std::unordered_map<CVehicle*, WheelRestZ> g_wheelRestZ;

void ApplySuspensionRotation(RwFrame* frame, float angle)
{
    if (!frame) return;

    auto& entry = g_suspMatrices[frame];
    if (!entry.stored)
    {
        entry.mat = *RwFrameGetMatrix(frame);
        entry.stored = true;
    }

    RwMatrix* mat = RwFrameGetMatrix(frame);
    *mat = entry.mat;

    RwV3d axis = { 0.0f, 1.0f, 0.0f };
    RwMatrixRotate(mat, &axis, angle, rwCOMBINEPRECONCAT);

    RwMatrixUpdate(mat);
    RwFrameUpdateObjects(frame);
}

struct SuspInfo
{
    int wheel;
    float sign;
};

SuspInfo GetSuspensionInfo(const char* name)
{
    if (!name) return { -1, 1.0f };

    // Using the signs you found work
    if (_strnicmp(name, "suspension_lf", 13) == 0) return { 0,  1.0f };
    if (_strnicmp(name, "suspension_rf", 13) == 0) return { 2, -1.0f };
    if (_strnicmp(name, "suspension_lr", 13) == 0) return { 1,  1.0f };
    if (_strnicmp(name, "suspension_lb", 13) == 0) return { 1,  1.0f };
    if (_strnicmp(name, "suspension_rr", 13) == 0) return { 3, -1.0f };
    if (_strnicmp(name, "suspension_rb", 13) == 0) return { 3, -1.0f };

    return { -1, 1.0f };
}

eCarNodes WheelToNode(int wheel)
{
    switch (wheel)
    {
    case 0: return CAR_WHEEL_LF;
    case 1: return CAR_WHEEL_LB;
    case 2: return CAR_WHEEL_RF;
    case 3: return CAR_WHEEL_RB;
    default: return CAR_NODE_NONE;
    }
}

class SuspensionRotate
{
public:
    SuspensionRotate()
    {
        Events::vehicleDtorEvent.before += [](CVehicle* vehicle)
        {
            g_wheelRestZ.erase(vehicle);
        };

        Events::vehicleRenderEvent.before += [](CVehicle* vehicle)
        {
            if (!vehicle || !vehicle->m_pRwClump)
                return;

            if (vehicle->m_nVehicleClass != VEHICLE_AUTOMOBILE &&
                vehicle->m_nVehicleClass != VEHICLE_MTRUCK &&
                vehicle->m_nVehicleClass != VEHICLE_QUAD)
                return;

            CAutomobile* autoMobile = static_cast<CAutomobile*>(vehicle);

            WheelRestZ& rest = g_wheelRestZ[vehicle];
            if (!rest.initialized)
            {
                for (int i = 0; i < 4; ++i)
                {
                    RwFrame* wheelFrame = autoMobile->m_aCarNodes[WheelToNode(i)];
                    if (wheelFrame)
                        rest.z[i] = RwFrameGetMatrix(wheelFrame)->pos.z;
                    else
                        rest.z[i] = 0.0f;
                }
                rest.initialized = true;
            }

            struct Data {
                CAutomobile* autoMobile;
                WheelRestZ* rest;
            } data = { autoMobile, &rest };

            RpClumpForAllAtomics(vehicle->m_pRwClump,
                [](RpAtomic* atomic, void* userdata) -> RpAtomic*
                {
                    Data* d = static_cast<Data*>(userdata);
                    if (!atomic) return atomic;

                    RwFrame* frame = RpAtomicGetFrame(atomic);
                    if (!frame) return atomic;

                    char* name = GetFrameNodeName(frame);
                    if (!name || _strnicmp(name, "suspension_", 11) != 0)
                        return atomic;

                    SuspInfo info = GetSuspensionInfo(name);
                    if (info.wheel < 0) return atomic;

                    RwFrame* wheelFrame = d->autoMobile->m_aCarNodes[WheelToNode(info.wheel)];
                    if (!wheelFrame) return atomic;

                    float currentZ = RwFrameGetMatrix(wheelFrame)->pos.z;
                    float restZ = d->rest->z[info.wheel];

                    float delta = currentZ - restZ;
                    float angle = delta * ROTATION_PER_UNIT * info.sign;

                    ApplySuspensionRotation(frame, angle);

                    return atomic;
                },
                &data);
        };
    }
} suspensionRotate;
