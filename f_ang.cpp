#include "plugin.h"
#include "CAutomobile.h"
#include "NodeName.h"
#include <unordered_map>
#include <cstring>
#include <cstdlib>

using namespace plugin;

namespace AngleOverride
{
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

    static void CollectParents(RwFrame* frame, std::unordered_map<RwFrame*, RwFrame*>& parentOf)
    {
        if (!frame) return;

        for (RwFrame* child = frame->child; child; child = child->next)
        {
            parentOf[child] = frame;
            CollectParents(child, parentOf);
        }
    }

    static std::unordered_map<CVehicle*, bool> g_processedVehicles;

    static void ScanVehicle(CAutomobile* automobile)
    {
        CVehicle* vehicle = static_cast<CVehicle*>(automobile);
        if (g_processedVehicles[vehicle]) return;

        if (!vehicle->m_pRwClump)
        {
            g_processedVehicles[vehicle] = true; // nothing to scan, don't retry every frame
            return;
        }

        RwFrame* rootFrame = reinterpret_cast<RwFrame*>(vehicle->m_pRwClump->object.parent);

        std::unordered_map<RwFrame*, RwFrame*> parentOf;
        CollectParents(rootFrame, parentOf);

        for (int node = CAR_NODE_NONE; node < CAR_NUM_NODES; ++node)
        {
            RwFrame* dummyFrame = automobile->m_aCarNodes[node];
            if (!dummyFrame) continue;

            int doorIndex;
            if (!CarNodeToDoorIndex(static_cast<eCarNodes>(node), doorIndex))
                continue;

            auto it = parentOf.find(dummyFrame);
            RwFrame* parentFrame = (it != parentOf.end()) ? it->second : nullptr;

            float radians;
            if (TryGetAngleOverrideRadians(parentFrame, radians))
            {
                automobile->m_doors[doorIndex].m_fOpenAngle = radians;
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
                AngleOverride::ScanVehicle(static_cast<CAutomobile*>(vehicle));
            };

        Events::vehicleDtorEvent.before += [](CVehicle* vehicle)
            {
                AngleOverride::g_processedVehicles.erase(vehicle);
            };
    }
} angleOverridePlugin;
