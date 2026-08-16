#include "plugin.h"
#include "CAutomobile.h"
#include "CHeli.h"
#include "NodeName.h"
#include <Windows.h>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <cstdint>

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
        default: return false;
        }
    }

    static int MaxNodeForVehicle(bool isHeli)
    {
        return isHeli ? (HELI_DOOR_LR + 1) : CAR_NUM_NODES;
    }

    static bool TryGetSlideOverride(const char* parentName, CVector& outOffset)
    {
        if (strncmp(parentName, "f_sld_", 6) != 0)
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

    static constexpr float kSmallAxisSpeedRatio = 0.4f;

    struct AxisSlices
    {
        float sliceStart[3] = {};
        float sliceEnd[3] = {};
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
        if (n == 1)
        {
            result.sliceStart[active[0].idx] = 0.0f;
            result.sliceEnd[active[0].idx] = 1.0f;
            return result;
        }

        float r = std::max(smallAxisSpeedRatio, 0.001f);
        float leadMag = active[n - 1].mag;
        float sumSmallMag = 0.0f;
        for (int i = 0; i < n - 1; ++i)
            sumSmallMag += active[i].mag;

        float leadSpeedUnit = leadMag + sumSmallMag / r;
        float cursor = 0.0f;
        for (int i = 0; i < n - 1; ++i)
        {
            float duration = active[i].mag / (r * leadSpeedUnit);
            result.sliceStart[active[i].idx] = cursor;
            result.sliceEnd[active[i].idx] = cursor + duration;
            cursor += duration;
        }
        result.sliceStart[active[n - 1].idx] = cursor;
        result.sliceEnd[active[n - 1].idx] = 1.0f;
        return result;
    }

    struct SlideData
    {
        bool hasOverride[6] = {};
        CVector slideOffset[6] = {};
        CVector closedPosition[6] = {};
        AxisSlices slices[6] = {};
        bool scanned = false;
    };

    static std::unordered_map<CVehicle*, SlideData> g_slideData;
    static std::vector<CVehicle*> g_activeHelis;

    static bool IsHelicopter(CVehicle* vehicle)
    {
        if (!vehicle) return false;
        if (vehicle->m_nVehicleClass == VEHICLE_HELI)
            return true;

        switch (vehicle->m_nModelIndex)
        {
        case 417: case 425: case 447: case 469:
        case 487: case 488: case 497: case 548: case 563:
            return true;
        default:
            return false;
        }
    }

    static float ApplyAxisSlide(float closed, float offset, float ratio, float sliceStart, float sliceEnd)
    {
        if (offset == 0.0f) return closed;
        float t;
        if (ratio <= sliceStart) t = 0.0f;
        else if (ratio >= sliceEnd) t = 1.0f;
        else t = (ratio - sliceStart) / (sliceEnd - sliceStart);
        return closed + offset * t;
    }

    static eCarNodes DoorIndexToNode(int doorIndex)
    {
        switch (doorIndex)
        {
        case 0: return CAR_BONNET;
        case 1: return CAR_BOOT;
        case 2: return CAR_DOOR_LF;
        case 3: return CAR_DOOR_RF;
        case 4: return CAR_DOOR_LR;
        case 5: return CAR_DOOR_RR;
        default: return CAR_NODE_NONE;
        }
    }

    static void ApplySlideForDoor(CAutomobile* automobile, int doorIndex)
    {
        if (!automobile || doorIndex < 0 || doorIndex > 5)
            return;

        auto it = g_slideData.find(static_cast<CVehicle*>(automobile));
        if (it == g_slideData.end() || !it->second.scanned || !it->second.hasOverride[doorIndex])
            return;

        SlideData& data = it->second;
        eCarNodes node = DoorIndexToNode(doorIndex);
        if (node == CAR_NODE_NONE)
            return;

        RwFrame* frame = automobile->m_aCarNodes[node];
        if (!frame)
            return;

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

    static void ScanVehicle(CAutomobile* automobile, SlideData& data, bool isHeli)
    {
        CVehicle* vehicle = static_cast<CVehicle*>(automobile);
        if (data.scanned)
            return;

        if (!vehicle->m_pRwClump)
        {
            data.scanned = true;
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

            auto parentIt = parentOf.find(dummyFrame);
            if (parentIt == parentOf.end()) continue;

            char* parentName = GetFrameNodeName(parentIt->second);
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

    static void ApplySlideOverrides(CAutomobile* automobile, SlideData& data, bool isHeli)
    {
        (void)isHeli;
        for (int i = 0; i < 6; ++i)
        {
            if (data.hasOverride[i])
                ApplySlideForDoor(automobile, i);
        }
    }

    void Update(CAutomobile* automobile)
    {
        CVehicle* vehicle = static_cast<CVehicle*>(automobile);
        bool isHeli = IsHelicopter(vehicle);
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

    // ----------------------------------------------------------------
    // Ped entry path: CVehicle::ProcessOpenDoor is virtual slot 28,
    // confirmed directly from plugin-sdk's CVehicle.cpp itself:
    //   (*(void***)this)[28] is exactly how the SDK calls it.
    // We patch that slot in the vtable rather than scanning for a
    // hardcoded function address - the slot index is a more stable
    // target than one build's raw memory offset for the function body.
    // ----------------------------------------------------------------

    using ProcessOpenDoor_t = void(__thiscall*)(CVehicle*, CPed*, uint32_t, uint32_t, uint32_t, float);
    static ProcessOpenDoor_t Real_ProcessOpenDoor = nullptr;

    static void __fastcall ProcessOpenDoor_Hook(CVehicle* self, void*, CPed* ped,
        uint32_t doorComponentId, uint32_t arg2, uint32_t arg3, float arg4)
    {
        Real_ProcessOpenDoor(self, ped, doorComponentId, arg2, arg3, arg4);

        if (!self)
            return;
        if (self->m_nVehicleClass != VEHICLE_AUTOMOBILE &&
            self->m_nVehicleClass != VEHICLE_HELI)
            return;

        Update(static_cast<CAutomobile*>(self));
    }

    static void PatchProcessOpenDoorSlot(uintptr_t* vmt)
    {
        if (!vmt) return;

        const int slot = 28;
        DWORD oldProt;
        if (!VirtualProtect(&vmt[slot], sizeof(uintptr_t), PAGE_READWRITE, &oldProt))
            return;

        if (!Real_ProcessOpenDoor)
            Real_ProcessOpenDoor = reinterpret_cast<ProcessOpenDoor_t>(vmt[slot]);

        vmt[slot] = reinterpret_cast<uintptr_t>(&ProcessOpenDoor_Hook);

        VirtualProtect(&vmt[slot], sizeof(uintptr_t), oldProt, &oldProt);
    }

    static void InstallProcessOpenDoorHooks()
    {
        // US 1.0 HOODLUM - class vtable bases (build-specific)
        PatchProcessOpenDoorSlot(reinterpret_cast<uintptr_t*>(0x871120)); // CAutomobile
        PatchProcessOpenDoorSlot(reinterpret_cast<uintptr_t*>(0x871680)); // CHeli
        // If Real_ProcessOpenDoor is still null, vtable base differs -
        // hooks simply do nothing (no crash)
    }
}

class SlideDoorPlugin
{
public:
    SlideDoorPlugin()
    {
        Events::initRwEvent += []
            {
                SlideDoor::InstallProcessOpenDoorHooks();
            };

        Events::vehicleSetModelEvent += [](CVehicle* vehicle, int)
            {
                SlideDoor::OnVehicleModelChanged(vehicle);
            };

        Events::vehicleRenderEvent.before += [](CVehicle* vehicle)
            {
                if (vehicle->m_nVehicleClass != VEHICLE_AUTOMOBILE &&
                    vehicle->m_nVehicleClass != VEHICLE_HELI)
                    return;
                SlideDoor::Update(static_cast<CAutomobile*>(vehicle));
            };

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
