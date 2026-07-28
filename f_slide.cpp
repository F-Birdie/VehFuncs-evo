#include "plugin.h"
#include "CAutomobile.h"
#include "CHeli.h"
#include "CPools.h"
#include "NodeName.h"

#include <unordered_map>
#include <vector>
#include <algorithm>
#include <cstring>
#include <cstdlib>
#include <cmath>

using namespace plugin;

namespace SlideDoor
{
    static bool CarNodeToDoorIndex(eCarNodes node, int& outDoorIndex)
    {
        switch (node)
        {
        case CAR_BONNET:  outDoorIndex = 0; return true;
        case CAR_BOOT:    outDoorIndex = 1; return true;
        case CAR_DOOR_LF: outDoorIndex = 2; return true;
        case CAR_DOOR_RF: outDoorIndex = 3; return true;
        case CAR_DOOR_LR: outDoorIndex = 4; return true;
        case CAR_DOOR_RR: outDoorIndex = 5; return true;
        default:          return false;
        }
    }

    static int MaxNodeForVehicle(bool isHeli)
    {
        return isHeli ? (HELI_DOOR_LR + 1) : CAR_NUM_NODES;
    }

    // Parses "f_sld_<x>,<y>,<z>"
    static bool TryGetSlideOverride(const char* parentName, CVector& outOffset)
    {
        const char* prefix = "f_sld_";
        if (strncmp(parentName, prefix, 6) != 0)
            return false;

        const char* cursor = parentName + 6;
        char* end = nullptr;

        float x = strtof(cursor, &end);
        if (end == cursor || *end != ',') return false;
        cursor = end + 1;

        float y = strtof(cursor, &end);
        if (end == cursor || *end != ',') return false;
        cursor = end + 1;

        float z = strtof(cursor, &end);
        if (end == cursor) return false;

        outOffset = CVector(x, y, z);
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

    struct SlideData
    {
        bool    hasOverride[6] = {};
        CVector slideOffset[6] = {};
        CVector closedPosition[6] = {};
        float   maxAxisMag[6] = {};
        bool    scanned = false;
    };

    static std::unordered_map<CVehicle*, SlideData> g_slideData;
    static std::vector<CVehicle*> g_activeHelis;

    static bool IsHelicopter(CVehicle* vehicle)
    {
        if (!vehicle) return false;

        if (vehicle->m_nVehicleClass == VEHICLE_HELI)
            return true;

        // Correct list of all helicopter models
        switch (vehicle->m_nModelIndex)
        {
        case 417: // leviathn
        case 425: // hunter
        case 447: // seaspar
        case 465: // rcraider
        case 469: // sparrow
        case 487: // maverick
        case 488: // vcnmav
        case 497: // polmav
        case 501: // rcgoblin
        case 548: // cargobob
        case 563: // raindanc
            return true;
        default:
            return false;
        }
    }

    static void ScanVehicle(CAutomobile* automobile)
    {
        CVehicle* vehicle = static_cast<CVehicle*>(automobile);
        SlideData& data = g_slideData[vehicle];

        if (data.scanned)
            return;

        if (!vehicle->m_pRwClump)
            return; // try again next frame

        bool isHeli = IsHelicopter(vehicle);
        int maxNode = MaxNodeForVehicle(isHeli);

        RwFrame* rootFrame = reinterpret_cast<RwFrame*>(vehicle->m_pRwClump->object.parent);
        std::unordered_map<RwFrame*, RwFrame*> parentOf;
        CollectParents(rootFrame, parentOf);

        for (int node = CAR_NODE_NONE; node < maxNode; ++node)
        {
            RwFrame* dummyFrame = automobile->m_aCarNodes[node];
            if (!dummyFrame) continue;

            int doorIndex;
            if (!CarNodeToDoorIndex(static_cast<eCarNodes>(node), doorIndex))
                continue;

            auto it = parentOf.find(dummyFrame);
            if (it == parentOf.end()) continue;

            char* parentName = GetFrameNodeName(it->second);
            if (!parentName) continue;

            CVector offset;
            if (!TryGetSlideOverride(parentName, offset))
                continue;

            data.hasOverride[doorIndex] = true;
            data.slideOffset[doorIndex] = offset;
            data.maxAxisMag[doorIndex] = std::max({ std::fabs(offset.x), std::fabs(offset.y), std::fabs(offset.z) });

            RwV3d* pos = RwMatrixGetPos(RwFrameGetMatrix(dummyFrame));
            data.closedPosition[doorIndex] = CVector(pos->x, pos->y, pos->z);
        }

        data.scanned = true;
    }

    static float ApplyAxisSlide(float closed, float offset, float traveled)
    {
        if (offset == 0.0f) return closed;

        float mag = std::fabs(offset);
        float sign = (offset > 0.0f) ? 1.0f : -1.0f;
        float dist = std::min(traveled, mag) * sign;
        return closed + dist;
    }

    static void ApplySlideOverrides(CAutomobile* automobile, SlideData& data)
    {
        bool isHeli = IsHelicopter(automobile);
        int maxNode = MaxNodeForVehicle(isHeli);

        for (int node = CAR_NODE_NONE; node < maxNode; ++node)
        {
            int doorIndex;
            if (!CarNodeToDoorIndex(static_cast<eCarNodes>(node), doorIndex))
                continue;
            if (!data.hasOverride[doorIndex]) continue;

            RwFrame* frame = automobile->m_aCarNodes[node];
            if (!frame) continue;

            float ratio = automobile->m_doors[doorIndex].GetAngleOpenRatio();
            float traveled = ratio * data.maxAxisMag[doorIndex];

            const CVector& offset = data.slideOffset[doorIndex];
            const CVector& closed = data.closedPosition[doorIndex];

            CVector newPos(
                ApplyAxisSlide(closed.x, offset.x, traveled),
                ApplyAxisSlide(closed.y, offset.y, traveled),
                ApplyAxisSlide(closed.z, offset.z, traveled)
            );

            RwV3d rwPos{ newPos.x, newPos.y, newPos.z };
            RwMatrix* rwMat = RwFrameGetMatrix(frame);
            RwMatrixSetIdentity(rwMat);
            RwMatrixTranslate(rwMat, &rwPos, rwCOMBINEREPLACE);
            RwFrameUpdateObjects(frame);
        }
    }

    void Update(CAutomobile* automobile)
    {
        CVehicle* vehicle = static_cast<CVehicle*>(automobile);
        ScanVehicle(automobile);

        auto it = g_slideData.find(vehicle);
        if (it != g_slideData.end() && it->second.scanned)
            ApplySlideOverrides(automobile, it->second);
    }

    void OnVehicleModelChanged(CVehicle* vehicle)
    {
        g_slideData[vehicle] = SlideData{};

        if (IsHelicopter(vehicle))
        {
            if (std::find(g_activeHelis.begin(), g_activeHelis.end(), vehicle) == g_activeHelis.end())
                g_activeHelis.push_back(vehicle);
        }
    }

    void OnVehicleDestroyed(CVehicle* vehicle)
    {
        g_slideData.erase(vehicle);

        auto it = std::find(g_activeHelis.begin(), g_activeHelis.end(), vehicle);
        if (it != g_activeHelis.end())
            g_activeHelis.erase(it);
    }
}

class SlideDoorPlugin
{
public:
    SlideDoorPlugin()
    {
        Events::vehicleSetModelEvent += [](CVehicle* vehicle, int)
            {
                SlideDoor::OnVehicleModelChanged(vehicle);
            };

        // Cars / normal automobiles
        Events::vehicleRenderEvent.before += [](CVehicle* vehicle)
            {
                if (vehicle->m_nVehicleClass != VEHICLE_AUTOMOBILE &&
                    vehicle->m_nVehicleClass != VEHICLE_HELI)
                    return;

                SlideDoor::Update(static_cast<CAutomobile*>(vehicle));
            };

        // Helicopters only (tiny list, no full pool walk)
        Events::processScriptsEvent += []
            {
                for (CVehicle* vehicle : SlideDoor::g_activeHelis)
                {
                    if (vehicle)
                        SlideDoor::Update(static_cast<CAutomobile*>(vehicle));
                }
            };

        Events::vehicleDtorEvent.before += [](CVehicle* vehicle)
            {
                SlideDoor::OnVehicleDestroyed(vehicle);
            };
    }
} slideDoorPlugin;
