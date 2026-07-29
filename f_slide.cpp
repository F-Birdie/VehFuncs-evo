#include "plugin.h"
#include "CAutomobile.h"
#include "CHeli.h"
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

    // How fast the smaller axes move during their own stage, as a
    // fraction of the lead (largest-offset) axis's speed. 1.0 = all
    // axes share time proportional to their distance, i.e. equal speed.
    // Lower values make the small axes noticeably slower/lazier relative
    // to the big slide. This is the one dial to tune.
    static constexpr float kSmallAxisSpeedRatio = 0.4f;

    // Splits the door's 0..1 open ratio into consecutive, non-overlapping
    // stages - smallest offset moves first, largest (the "lead" axis)
    // moves last. Stage widths are solved so the small axes move at
    // kSmallAxisSpeedRatio times the lead axis's speed, while the whole
    // set of stages still always sums to exactly 1 - no clamping needed,
    // the formula can't produce a total over 1 for any ratio > 0.
    struct AxisSlices
    {
        float sliceStart[3] = { 0.0f, 0.0f, 0.0f };
        float sliceEnd[3] = { 0.0f, 0.0f, 0.0f };
    };

    static AxisSlices ComputeAxisSlices(const CVector& offset, float smallAxisSpeedRatio)
    {
        AxisSlices result;

        struct AxisEntry { int idx; float mag; };
        std::vector<AxisEntry> active;

        if (offset.x != 0.0f) active.push_back({ 0, std::fabs(offset.x) });
        if (offset.y != 0.0f) active.push_back({ 1, std::fabs(offset.y) });
        if (offset.z != 0.0f) active.push_back({ 2, std::fabs(offset.z) });

        if (active.empty()) return result;

        std::sort(active.begin(), active.end(),
            [](const AxisEntry& a, const AxisEntry& b) { return a.mag < b.mag; });

        int n = static_cast<int>(active.size());

        // Only one axis in use - it just gets the whole range, no "small
        // axis" concept applies.
        if (n == 1)
        {
            result.sliceStart[active[0].idx] = 0.0f;
            result.sliceEnd[active[0].idx] = 1.0f;
            return result;
        }

        float r = std::max(smallAxisSpeedRatio, 0.001f); // guard divide-by-zero
        float leadMag = active[n - 1].mag;

        float sumSmallMag = 0.0f;
        for (int i = 0; i < n - 1; ++i)
            sumSmallMag += active[i].mag;

        // Solved so lead-axis speed and small-axis speed = r * lead-speed
        // both come out consistent with all stages summing to 1.
        float leadSpeedUnit = leadMag + sumSmallMag / r;

        float cursor = 0.0f;
        for (int i = 0; i < n - 1; ++i)
        {
            float duration = active[i].mag / (r * leadSpeedUnit);
            result.sliceStart[active[i].idx] = cursor;
            result.sliceEnd[active[i].idx] = cursor + duration;
            cursor += duration;
        }

        // Lead axis takes the remainder, forced to end exactly at 1.0 to
        // avoid float drift leaving a tiny gap at full-open.
        result.sliceStart[active[n - 1].idx] = cursor;
        result.sliceEnd[active[n - 1].idx] = 1.0f;

        return result;
    }

    struct SlideData
    {
        bool       hasOverride[6] = {};
        CVector    slideOffset[6] = {};
        CVector    closedPosition[6] = {};
        AxisSlices slices[6] = {};
        bool       scanned = false;
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
      //  case 465: // rcraider
        case 469: // sparrow
        case 487: // maverick
        case 488: // vcnmav
        case 497: // polmav
      //  case 501: // rcgoblin
        case 548: // cargobob
        case 563: // raindanc
            return true;
        default:
            return false;
        }
    }

    // isHeli is now computed once by the caller (Update) and threaded
    // through, instead of ScanVehicle and ApplySlideOverrides each
    // re-running the same IsHelicopter() switch every frame.
    static void ScanVehicle(CAutomobile* automobile, SlideData& data, bool isHeli)
    {
        CVehicle* vehicle = static_cast<CVehicle*>(automobile);

        if (data.scanned)
            return;

        if (!vehicle->m_pRwClump)
        {
            data.scanned = true; // nothing to scan, don't retry every frame
            return;
        }

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
            data.slices[doorIndex] = ComputeAxisSlices(offset, kSmallAxisSpeedRatio);

            RwV3d* pos = RwMatrixGetPos(RwFrameGetMatrix(dummyFrame));
            data.closedPosition[doorIndex] = CVector(pos->x, pos->y, pos->z);
        }

        data.scanned = true;
    }

    static float ApplyAxisSlide(float closed, float offset, float ratio, float sliceStart, float sliceEnd)
    {
        if (offset == 0.0f) return closed;

        float t;
        if (ratio <= sliceStart)      t = 0.0f;
        else if (ratio >= sliceEnd)   t = 1.0f;
        else                          t = (ratio - sliceStart) / (sliceEnd - sliceStart);

        return closed + offset * t;
    }

    static void ApplySlideOverrides(CAutomobile* automobile, SlideData& data, bool isHeli)
    {
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

            const CVector& offset = data.slideOffset[doorIndex];
            const CVector& closed = data.closedPosition[doorIndex];
            const AxisSlices& slices = data.slices[doorIndex];

            CVector newPos(
                ApplyAxisSlide(closed.x, offset.x, ratio, slices.sliceStart[0], slices.sliceEnd[0]),
                ApplyAxisSlide(closed.y, offset.y, ratio, slices.sliceStart[1], slices.sliceEnd[1]),
                ApplyAxisSlide(closed.z, offset.z, ratio, slices.sliceStart[2], slices.sliceEnd[2])
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
        bool isHeli = IsHelicopter(vehicle);

        // Single lookup, reused for both the scan and the apply step,
        // instead of looking the vehicle up in the map twice.
        SlideData& data = g_slideData[vehicle];

        ScanVehicle(automobile, data, isHeli);

        if (data.scanned)
            ApplySlideOverrides(automobile, data, isHeli);
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
