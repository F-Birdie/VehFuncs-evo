/*
    AngleOverride - extension for VehFuncs-style vehicle dummy conventions
    Lets modelers override a door/boot/bonnet's opening angle by parenting
    its dummy under a node named "f_ang_<signed integer degrees>".

    Example hierarchy:
        chassis
        --f_ang_-90
        ---boot_dummy
        ----boot_ok
        ----boot_dam

    Built on https://github.com/DK22Pac/plugin-sdk
*/
#include "plugin.h"
#include "CAutomobile.h"
#include "NodeName.h"
#include <unordered_map>
#include <cstring>
#include <cstdlib>

using namespace plugin;

namespace AngleOverride
{
    // Index order inferred from the commented-out lines in plugin-sdk's
    // OpenDoorExample.cpp (BONNET, BOOT, DOOR_FRONT_LEFT, DOOR_FRONT_RIGHT,
    // DOOR_REAR_LEFT, DOOR_REAR_RIGHT = 0..5). We deliberately do NOT
    // redeclare eDoors here since the SDK already defines the real one
    // (used by CAutomobile::FixDoor/SetDoorDamage) - redeclaring caused
    // a type conflict. We just cast plain ints to the SDK's real eDoors.
    static bool CarNodeToDoorIndex(eCarNodes node, int& outDoorIndex)
    {
        switch (node)
        {
        case CAR_BONNET:   outDoorIndex = 0; return true;
        case CAR_BOOT:     outDoorIndex = 1; return true;
        case CAR_DOOR_LF:  outDoorIndex = 2; return true;
        case CAR_DOOR_RF:  outDoorIndex = 3; return true;
        case CAR_DOOR_LR:  outDoorIndex = 4; return true;
        case CAR_DOOR_RR:  outDoorIndex = 5; return true;
        default:           return false;
        }
    }

    // Parses "f_ang_<int>" (integer degrees, signed) off a frame's name.
    static bool TryGetAngleOverrideRadians(RwFrame* parentFrame, float& outRadians)
    {
        if (!parentFrame) return false;

        char* parentName = GetFrameNodeName(parentFrame);
        if (!parentName) return false;

        const char* prefix = "f_ang_";
        const size_t prefixLen = 6;
        if (strncmp(parentName, prefix, prefixLen) != 0) return false;

        char* end = nullptr;
        long degrees = strtol(parentName + prefixLen, &end, 10);
        if (end == parentName + prefixLen) return false; // no digits parsed

        outRadians = degrees * (3.14159265f / 180.0f);
        return true;
    }

    // RwFrame has no parent pointer - only child (first child) and
    // next (next sibling). To find a frame's parent we have to search
    // down from the clump's root frame ourselves.
    static RwFrame* FindParentFrame(RwFrame* searchRoot, RwFrame* target)
    {
        if (!searchRoot) return nullptr;

        for (RwFrame* child = searchRoot->child; child; child = child->next)
        {
            if (child == target) return searchRoot;

            RwFrame* found = FindParentFrame(child, target);
            if (found) return found;
        }
        return nullptr;
    }

    static std::unordered_map<CVehicle*, bool> g_processedVehicles;

    static void ScanVehicle(CAutomobile* automobile)
    {
        CVehicle* vehicle = reinterpret_cast<CVehicle*>(automobile);
        if (g_processedVehicles[vehicle]) return;

        if (!vehicle->m_pRwClump)
        {
            g_processedVehicles[vehicle] = true; // nothing to scan, don't retry every frame
            return;
        }

        RwFrame* rootFrame = reinterpret_cast<RwFrame*>(vehicle->m_pRwClump->object.parent);

        for (int node = CAR_NODE_NONE; node < CAR_NUM_NODES; ++node)
        {
            RwFrame* dummyFrame = automobile->m_aCarNodes[node];
            if (!dummyFrame) continue;

            int doorIndex;
            if (!CarNodeToDoorIndex(static_cast<eCarNodes>(node), doorIndex))
                continue;

            RwFrame* parentFrame = FindParentFrame(rootFrame, dummyFrame);

            float radians;
            if (TryGetAngleOverrideRadians(parentFrame, radians))
            {
                automobile->m_doors[doorIndex].m_fOpenAngle = radians;

                // If the door swings the wrong direction instead of just
                // the wrong amount, uncomment and test this too:
                // automobile->m_doors[doorIndex].m_nDirn = (radians < 0.0f) ? -1 : 1;
            }
        }

        g_processedVehicles[vehicle] = true;
    }
}

class AngleOverridePlugin
{
public:
    AngleOverridePlugin()
    {
        Events::vehicleSetModelEvent += [](CVehicle* vehicle, int modelId)
            {
                AngleOverride::g_processedVehicles[vehicle] = false;
            };

        Events::vehicleRenderEvent.before += [](CVehicle* vehicle)
            {
                if (vehicle->m_nVehicleClass != VEHICLE_AUTOMOBILE) return;
                AngleOverride::ScanVehicle(reinterpret_cast<CAutomobile*>(vehicle));
            };

        Events::vehicleDtorEvent.before += [](CVehicle* vehicle)
            {
                AngleOverride::g_processedVehicles.erase(vehicle);
            };
    }
} angleOverridePlugin;
