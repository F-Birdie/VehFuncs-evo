#include "plugin.h"
#include "CAutomobile.h"
#include "NodeName.h"
#include <unordered_map>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <algorithm>

using namespace plugin;

namespace SlideDoor
{
    // Same mapping as the verified-working AngleOverride.cpp - do not
    // change this without re-testing, since door-index order does not
    // match CAutomobile's constructor init order (confirmed by the
    // RR/RF mixup).
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

    // Parses "f_sld_<x>,<y>,<z>" - signed decimals, 2 decimal places by
    // convention (e.g. "f_sld_-0.06,-1.06,0.00").
    static bool TryGetSlideOverride(const char* parentName, CVector& outOffset)
    {
        const char* prefix = "f_sld_";
        const size_t prefixLen = 6;
        if (strncmp(parentName, prefix, prefixLen) != 0) return false;

        const char* cursor = parentName + prefixLen;
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
        // Largest of |x|,|y|,|z| in slideOffset[i]. Used as the shared
        // "distance travelled" rate so every axis moves at the same
        // speed - the axis with the smallest offset just reaches its
        // target early and holds, rather than crawling the whole swing.
        float   maxAxisMag[6] = {};
        bool    scanned = false;
    };

    static std::unordered_map<CVehicle*, SlideData> g_slideData;

    static void ScanVehicle(CAutomobile* automobile)
    {
        CVehicle* vehicle = static_cast<CVehicle*>(automobile);
        SlideData& data = g_slideData[vehicle];

        if (data.scanned) return;

        if (!vehicle->m_pRwClump)
        {
            data.scanned = true;
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
            if (!parentFrame) continue;

            char* parentName = GetFrameNodeName(parentFrame);
            if (!parentName) continue;

            CVector offset;
            if (!TryGetSlideOverride(parentName, offset)) continue;

            data.hasOverride[doorIndex] = true;
            data.slideOffset[doorIndex] = offset;
            data.maxAxisMag[doorIndex] = std::max({ std::fabs(offset.x), std::fabs(offset.y), std::fabs(offset.z) });

            RwV3d* pos = RwMatrixGetPos(RwFrameGetMatrix(dummyFrame));
            data.closedPosition[doorIndex] = CVector(pos->x, pos->y, pos->z);
        }

        data.scanned = true;
    }

    // Moves a single axis toward its target offset at the shared
    // "traveled" rate, clamping once it reaches its own total distance.
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
        for (int node = CAR_NODE_NONE; node < CAR_NUM_NODES; ++node)
        {
            int doorIndex;
            if (!CarNodeToDoorIndex(static_cast<eCarNodes>(node), doorIndex))
                continue;

            if (!data.hasOverride[doorIndex]) continue;

            RwFrame* frame = automobile->m_aCarNodes[node];
            if (!frame) continue;

            float ratio = automobile->m_doors[doorIndex].GetAngleOpenRatio();

            const CVector& offset = data.slideOffset[doorIndex];
            const CVector& closed = data.closedPosition[doorIndex];
            float traveled = ratio * data.maxAxisMag[doorIndex];

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

    static std::unordered_map<CVehicle*, bool> g_needsRescan;

    void Update(CAutomobile* automobile)
    {
        CVehicle* vehicle = static_cast<CVehicle*>(automobile);
        ScanVehicle(automobile);
        ApplySlideOverrides(automobile, g_slideData[vehicle]);
    }

    void OnVehicleModelChanged(CVehicle* vehicle)
    {
        g_slideData[vehicle] = SlideData{}; // force rescan
    }

    void OnVehicleDestroyed(CVehicle* vehicle)
    {
        g_slideData.erase(vehicle);
    }
}

class SlideDoorPlugin
{
public:
    SlideDoorPlugin()
    {
        Events::vehicleSetModelEvent += [](CVehicle* vehicle, int modelId)
            {
                SlideDoor::OnVehicleModelChanged(vehicle);
            };

        Events::vehicleRenderEvent.before += [](CVehicle* vehicle)
            {
                if (vehicle->m_nVehicleClass != VEHICLE_AUTOMOBILE) return;
                SlideDoor::Update(static_cast<CAutomobile*>(vehicle));
            };

        Events::vehicleDtorEvent.before += [](CVehicle* vehicle)
            {
                SlideDoor::OnVehicleDestroyed(vehicle);
            };
    }
} slideDoorPlugin;
