#include "PxPhysicsAPI.h"
#include <cstdint>
#include "physx_generated.hpp"
#include <iostream>

#ifdef _WIN32
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT
#endif

PxDefaultAllocator gAllocator;
PxDefaultErrorCallback gErrorCallback;

extern "C" {
    typedef void (*CollisionCallback)(void *, PxContactPairHeader const *, PxContactPair const *, PxU32);
}

extern "C" {
    typedef PxU16 (*SimulationFilterShader)(
        PxFilterObjectAttributes* attributes0,
        PxFilterData* filterData0,
        PxFilterObjectAttributes* attributes1,
        PxFilterData* filterData1,
        PxPairFlags* pairFlags);
}

struct FilterShaderHandle
{
    SimulationFilterShader shader;
};


PxU16 DefaultSimulationFilterShader(PxFilterObjectAttributes* attributes0,
                                    PxFilterData* filterData0,
                                    PxFilterObjectAttributes* attributes1,
                                    PxFilterData* filterData1,
                                    PxPairFlags* pairFlags)
{
    return PxDefaultSimulationFilterShader(*attributes0, *filterData0, *attributes1, *filterData1, *pairFlags, nullptr, 0);
}

PxFilterFlags FilterShaderTrampoline(PxFilterObjectAttributes attributes0,
                                     PxFilterData filterData0,
                                     PxFilterObjectAttributes attributes1,
                                     PxFilterData filterData1,
                                     PxPairFlags& pairFlags,
                                     const void* constantBlock,
                                     PxU32 constantBlockSize)
{
    // std::cout << "attributes0 " << attributes0 << std::endl;
    // std::cout << "filterData0 " << filterData0.word0 << " " << filterData0.word1 << " " << filterData0.word2 << " " << filterData0.word3 << std::endl;
    // std::cout << "attributes1 " << attributes1 << std::endl;
    // std::cout << "filterData1 " << filterData1.word0 << " " << filterData1.word1 << " " << filterData1.word2 << " " << filterData1.word3 << std::endl;
    // std::cout << "pairFlags " << &pairFlags << std::endl;
    // std::cout << "constantBlock " << constantBlock << std::endl;
    // std::cout << "constantBlockSize " << constantBlockSize << std::endl;

    FilterShaderHandle* handle = static_cast<FilterShaderHandle*>(const_cast<void*>(constantBlock));

    // We return a u16 since PxFilterFlags is a complex type and C++ wants it to be returned on the stack,
    // but Rust thinks it's simple due to the codegen and wants to return it in EAX.
    return PxFilterFlags{handle->shader(&attributes0, &filterData0, &attributes1, &filterData1, &pairFlags)};
}

PxFilterFlags FilterShaderTrampolineWithDefault(PxFilterObjectAttributes attributes0,
                                     PxFilterData filterData0,
                                     PxFilterObjectAttributes attributes1,
                                     PxFilterData filterData1,
                                     PxPairFlags& pairFlags,
                                     const void* constantBlock,
                                     PxU32 constantBlockSize)
{
    // Let the default handler set the pair flags, but ignore the collision filtering
    PxDefaultSimulationFilterShader(attributes0, filterData0, attributes1, filterData1, pairFlags, nullptr, 0);

    FilterShaderHandle* handle = static_cast<FilterShaderHandle*>(const_cast<void*>(constantBlock));

    // We return a u16 since PxFilterFlags is a complex type and C++ wants it to be returned on the stack,
    // but Rust thinks it's simple due to the codegen and wants to return it in EAX.
    // PxU16 value = callback(attributes0, filterData0, attributes1, filterData1, pairFlags, nullptr, 0);
    return PxFilterFlags{handle->shader(&attributes0, &filterData0, &attributes1, &filterData1, &pairFlags)};
}


extern "C" {
    using CollisionCallback = void (*)(void *, PxContactPairHeader const *, PxContactPair const *, PxU32);
    using TriggerCallback = void (*)(void *, PxTriggerPair const *, PxU32);
    using ConstraintBreakCallback = void (*)(void *, PxConstraintInfo const *, PxU32);
    using WakeSleepCallback = void (*)(void *, PxActor **const, PxU32, bool);
    using AdvanceCallback = void (*)(void *, const PxRigidBody *const *, const PxTransform *const, PxU32);
}

struct SimulationEventCallbackInfo {
    // Callback for collision events.
    CollisionCallback collisionCallback = nullptr;
    void *collisionUserData = nullptr;
    // Callback for trigger shape events (an object entered or left a trigger shape).
    TriggerCallback triggerCallback = nullptr;
    void *triggerUserData = nullptr;
    // Callback for when a constraint breaks (such as a joint with a force limit)
    ConstraintBreakCallback constraintBreakCallback = nullptr;
    void *constraintBreakUserData = nullptr;
    // Callback for when an object falls asleep or is awoken.
    WakeSleepCallback wakeSleepCallback = nullptr;
    void *wakeSleepUserData = nullptr;
    // Callback to get the next pose early for objects (if flagged with eENABLE_POSE_INTEGRATION_PREVIEW).
    AdvanceCallback advanceCallback = nullptr;
    void *advanceUserData = nullptr;
};

class SimulationEventTrampoline : public PxSimulationEventCallback
{
  public:
    SimulationEventTrampoline(const SimulationEventCallbackInfo *callbacks) : mCallbacks(*callbacks) {}

    // Collisions
    void onContact(const PxContactPairHeader &pairHeader, const PxContactPair *pairs, PxU32 nbPairs) override {
        if (mCallbacks.collisionCallback) {
            mCallbacks.collisionCallback(mCallbacks.collisionUserData, &pairHeader, pairs, nbPairs);
        }
    }

    // Triggers
    void onTrigger(PxTriggerPair *pairs, PxU32 count) override {
        if (mCallbacks.triggerCallback) {
            mCallbacks.triggerCallback(mCallbacks.triggerUserData, pairs, count);
        }
    }

    // Constraint breaks
    void onConstraintBreak(PxConstraintInfo *constraints, PxU32 count) override {
        if (mCallbacks.constraintBreakCallback) {
            mCallbacks.constraintBreakCallback(mCallbacks.constraintBreakUserData, constraints, count);
        }
    }

    // Wake/Sleep (combined for convenience)
    void onWake(PxActor **actors, PxU32 count) override {
        if (mCallbacks.wakeSleepCallback) {
            mCallbacks.wakeSleepCallback(mCallbacks.wakeSleepUserData, actors, count, true);
        }
    }
    void onSleep(PxActor **actors, PxU32 count) override {
        if (mCallbacks.wakeSleepCallback) {
            mCallbacks.wakeSleepCallback(mCallbacks.wakeSleepUserData, actors, count, false);
        }
    }

    // Advance
    void onAdvance(const PxRigidBody *const * bodyBuffer, const PxTransform *poseBuffer, const PxU32 count) override {
        if (mCallbacks.advanceCallback) {
            mCallbacks.advanceCallback(mCallbacks.advanceUserData, bodyBuffer, poseBuffer, count);
        }
    }

    SimulationEventCallbackInfo mCallbacks;
};

class RaycastFilterCallback : public PxQueryFilterCallback
{
  public:
    explicit RaycastFilterCallback(PxRigidActor *actor) : mActor(actor) {}

    PxRigidActor *mActor;

    virtual PxQueryHitType::Enum preFilter(const PxFilterData &, const PxShape *shape, const PxRigidActor *actor, PxHitFlags &)
    {
        if (mActor == actor)
        {
            return PxQueryHitType::eNONE;
        }
        else
        {
            return PxQueryHitType::eBLOCK;
        }
    }

    virtual PxQueryHitType::Enum postFilter(const PxFilterData &, const PxQueryHit &)
    {
        return PxQueryHitType::eNONE;
    }
};

extern "C" {
    typedef int32_t (*RaycastHitCallback)(const PxRigidActor *actor, const PxFilterData *filterData, const PxShape *shape, uint32_t hitFlags, const void *userData);
    typedef int32_t (*PostFilterCallback)(const PxFilterData *filterData, const PxQueryHit* hit, const void *userData);
}

PxQueryHitType::Enum sanitize_hit_type(int32_t hit_type) {
    switch (hit_type) {
        case PxQueryHitType::eNONE:
        case PxQueryHitType::eTOUCH:
        case PxQueryHitType::eBLOCK: return (PxQueryHitType::Enum)hit_type;
        default: return PxQueryHitType::eNONE;
    }
}

class RaycastFilterTrampoline : public PxQueryFilterCallback
{
  public:
    RaycastFilterTrampoline(RaycastHitCallback callback, const void *userdata)
        : mCallback(callback), mUserData(userdata) {}

    RaycastHitCallback mCallback;
    const void *mUserData;

    virtual PxQueryHitType::Enum preFilter(const PxFilterData &filterData, const PxShape *shape, const PxRigidActor *actor, PxHitFlags &hitFlags)
    {
        return sanitize_hit_type(mCallback(actor, &filterData, shape, (uint32_t)hitFlags, mUserData));
    }

    virtual PxQueryHitType::Enum postFilter(const PxFilterData &, const PxQueryHit &)
    {
        return PxQueryHitType::eNONE;
    }
};


class RaycastFilterPrePostTrampoline : public PxQueryFilterCallback
{
  public:
    RaycastFilterPrePostTrampoline(RaycastHitCallback preFilter, PostFilterCallback postFilter, const void *userdata)
        : mPreFilter(preFilter), mPostFilter(postFilter), mUserData(userdata) {}

    RaycastHitCallback mPreFilter;
    PostFilterCallback mPostFilter;
    
    const void *mUserData;

    virtual PxQueryHitType::Enum preFilter(const PxFilterData &filterData, const PxShape *shape, const PxRigidActor *actor, PxHitFlags &hitFlags)
    {
        return sanitize_hit_type(mPreFilter(actor, &filterData, shape, (uint32_t)hitFlags, mUserData));

    }

    virtual PxQueryHitType::Enum postFilter(const PxFilterData &filterData, const PxQueryHit &hit)
    {
        return sanitize_hit_type(mPostFilter(&filterData, &hit, mUserData));
    }
};

extern "C" {
    typedef PxAgain (*RaycastHitProcessTouchesCallback)(const PxRaycastHit *buffer, PxU32 nbHits, void *userdata);
    typedef PxAgain (*SweepHitProcessTouchesCallback)(const PxSweepHit *buffer, PxU32 nbHits, void *userdata);
    typedef PxAgain (*OverlapHitProcessTouchesCallback)(const PxOverlapHit *buffer, PxU32 nbHits, void *userdata);
    typedef void (*HitFinalizeQueryCallback)(void *userdata);
}

class RaycastHitCallbackTrampoline : public PxRaycastCallback
{
  public:
    RaycastHitCallbackTrampoline(
        RaycastHitProcessTouchesCallback processTouchesCallback,
        HitFinalizeQueryCallback finalizeQueryCallback,
        PxRaycastHit *touchesBuffer,
        PxU32 numTouches,
        void *userdata)
        : PxRaycastCallback(touchesBuffer, numTouches),
          mProcessTouchesCallback(processTouchesCallback),
          mFinalizeQueryCallback(finalizeQueryCallback),
          mUserData(userdata) {}

    RaycastHitProcessTouchesCallback mProcessTouchesCallback;
    HitFinalizeQueryCallback mFinalizeQueryCallback;
    void *mUserData;

    PxAgain processTouches(const PxRaycastHit *buffer, PxU32 nbHits) override
    {
        return mProcessTouchesCallback(buffer, nbHits, mUserData);
    }

    void finalizeQuery() override
    {
        mFinalizeQueryCallback(mUserData);
    }
};

class SweepHitCallbackTrampoline : public PxSweepCallback
{
  public:
    SweepHitCallbackTrampoline(
        SweepHitProcessTouchesCallback processTouchesCallback,
        HitFinalizeQueryCallback finalizeQueryCallback,
        PxSweepHit *touchesBuffer,
        PxU32 numTouches,
        void *userdata)
        : PxSweepCallback(touchesBuffer, numTouches),
          mProcessTouchesCallback(processTouchesCallback),
          mFinalizeQueryCallback(finalizeQueryCallback),
          mUserData(userdata) {}

    SweepHitProcessTouchesCallback mProcessTouchesCallback;
    HitFinalizeQueryCallback mFinalizeQueryCallback;
    void *mUserData;

    PxAgain processTouches(const PxSweepHit *buffer, PxU32 nbHits) override
    {
        return mProcessTouchesCallback(buffer, nbHits, mUserData);
    }

    void finalizeQuery() override
    {
        mFinalizeQueryCallback(mUserData);
    }
};

class OverlapHitCallbackTrampoline : public PxOverlapCallback
{
  public:
    OverlapHitCallbackTrampoline(
        OverlapHitProcessTouchesCallback processTouchesCallback,
        HitFinalizeQueryCallback finalizeQueryCallback,
        PxOverlapHit *touchesBuffer,
        PxU32 numTouches,
        void *userdata)
        : PxOverlapCallback(touchesBuffer, numTouches),
          mProcessTouchesCallback(processTouchesCallback),
          mFinalizeQueryCallback(finalizeQueryCallback),
          mUserData(userdata) {}

    OverlapHitProcessTouchesCallback mProcessTouchesCallback;
    HitFinalizeQueryCallback mFinalizeQueryCallback;
    void *mUserData;

    PxAgain processTouches(const PxOverlapHit *buffer, PxU32 nbHits) override
    {
        return mProcessTouchesCallback(buffer, nbHits, mUserData);
    }

    void finalizeQuery() override
    {
        mFinalizeQueryCallback(mUserData);
    }
};

extern "C" {
    typedef void * (*AllocCallback)(uint64_t size, const char *typeName, const char *filename, int line, void *userdata);
    typedef void (*DeallocCallback)(void *ptr, void *userdata);
}

class CustomAllocatorTrampoline : public PxAllocatorCallback {
public:
    CustomAllocatorTrampoline(AllocCallback allocCb, DeallocCallback deallocCb, void *userdata)
        : mAllocCallback(allocCb), mDeallocCallback(deallocCb), mUserData(userdata) {}

    void *allocate(size_t size, const char *typeName, const char *filename, int line)
    {
        return mAllocCallback((uint64_t)size, typeName, filename, line, mUserData);
    }

    virtual void deallocate(void* ptr)
    {
        mDeallocCallback(ptr, mUserData);
    }

private:
    AllocCallback mAllocCallback;
    DeallocCallback mDeallocCallback;
public:
    void *mUserData;
};

extern "C" {
    typedef void * (*ZoneStartCallback)(const char *typeName, bool detached, uint64_t context  , void *userdata);
    typedef void (*ZoneEndCallback)(void* profilerData, const char *typeName, bool detached, uint64_t context , void *userdata);
}

class CustomProfilerTrampoline : public PxProfilerCallback {
public:
    CustomProfilerTrampoline(ZoneStartCallback startCb, ZoneEndCallback endCb, void *userdata)
        : mStartCallback(startCb), mEndCallback(endCb), mUserData(userdata) {
    }

	virtual void* zoneStart(const char* eventName, bool detached, uint64_t contextId) override
	{
		return mStartCallback(eventName, detached, contextId, mUserData);
	}

	virtual void zoneEnd(void* profilerData, const char* eventName, bool detached, uint64_t contextId) override
	{
		return mEndCallback(profilerData, eventName, detached, contextId, mUserData);
	}

private:
    ZoneStartCallback mStartCallback;
    ZoneEndCallback mEndCallback;
public:
    void *mUserData;
};

extern "C" {
    using ErrorCallback = void (*)(int code, const char* message, const char* file, int line, void* userdata);
}

class ErrorTrampoline : public PxErrorCallback {
public:
    ErrorTrampoline(ErrorCallback errorCb, void* userdata)
        : mErrorCallback(errorCb), mUserdata(userdata)
	{}

    void reportError(PxErrorCode::Enum code, const char* message, const char* file, int line) override {
        mErrorCallback(code, message, file, line, mUserdata);
    }

private:
    ErrorCallback mErrorCallback = nullptr;
    void* mUserdata = nullptr;
};

extern "C" {
    using AssertHandler = void (*)(const char* expr, const char* file, int line, bool* should_ignore, void* userdata);
}

class AssertTrampoline : public PxAssertHandler {
public:
    AssertTrampoline(AssertHandler onAssert, void* userdata)
        : mAssertHandler(onAssert), mUserdata(userdata)
	{}

	virtual void operator()(const char* exp, const char* file, int line, bool& ignore) override final {
        mAssertHandler(exp, file, line, &ignore, mUserdata);
    }

private:
    AssertHandler mAssertHandler = nullptr;
    void* mUserdata = nullptr;
};

extern "C" {
    using ShapeHitCallback = void (*)(void *, PxControllerShapeHit const *);
    using ControllerHitCallback = void (*)(void *, PxControllersHit const *);
    using ObstacleHitCallback = void (*)(void *, PxControllerObstacleHit const *);
}

struct UserControllerHitReportInfo {
    ShapeHitCallback shapeHitCallback = nullptr;
    void* shapeHitUserData = nullptr;
    ControllerHitCallback controllerHitCallback = nullptr;
    void* controllerHitUserData = nullptr;
    ObstacleHitCallback obstacleHitCallback = nullptr;
    void* obstacleHitUserData = nullptr;
};

class UserControllerHitReport : public PxUserControllerHitReport {
public:
    UserControllerHitReport(const UserControllerHitReportInfo *callbacks) : mCallbacks(*callbacks) {}

    void onShapeHit(const PxControllerShapeHit& hit) override {
        if (mCallbacks.shapeHitCallback) {
            mCallbacks.shapeHitCallback(mCallbacks.shapeHitUserData, &hit);
        }
    }

    void onControllerHit(const PxControllersHit& hit) override {
        if (mCallbacks.controllerHitCallback) {
            mCallbacks.controllerHitCallback(mCallbacks.controllerHitUserData, &hit);
        }
    }

    void onObstacleHit(const PxControllerObstacleHit& hit) override {
        if (mCallbacks.obstacleHitCallback) {
            mCallbacks.obstacleHitCallback(mCallbacks.obstacleHitUserData, &hit);
        }
    }

    UserControllerHitReportInfo mCallbacks;
};

extern "C" {
    using ShapeBehaviorCallback = PxControllerBehaviorFlags (*)(void* data, const PxShape* shape, const PxActor* actor);
    using CharacterBehaviorCallback = PxControllerBehaviorFlags (*)(void* data, const PxController* controller);
    using ObstacleBehaviorCallback = PxControllerBehaviorFlags (*)(void* data, const PxObstacle* obstacle);
}

struct ControllerBehaviorCallbackInfo {
    ShapeBehaviorCallback shapeBehaviorCallback = nullptr;
    void* shapeBehaviorUserData = nullptr;
    CharacterBehaviorCallback controllerBehaviorCallback = nullptr;
    void* controllerBehaviorUserData = nullptr;
    ObstacleBehaviorCallback obstacleBehaviorCallback = nullptr;
    void* obstacleBehaviorUserData = nullptr;
};

class ControllerBehaviorCallback : public PxControllerBehaviorCallback {
public:
    ControllerBehaviorCallback(const ControllerBehaviorCallbackInfo *callbacks) : mCallbacks(*callbacks) {}

    PxControllerBehaviorFlags getBehaviorFlags(const PxShape& shape, const PxActor& actor) override {
        if (mCallbacks.shapeBehaviorCallback) {
            return mCallbacks.shapeBehaviorCallback(mCallbacks.shapeBehaviorUserData, &shape, &actor);
        }
        return static_cast<PxControllerBehaviorFlags>(0);
    }

    PxControllerBehaviorFlags getBehaviorFlags(const PxController& controller) override {
        if (mCallbacks.controllerBehaviorCallback) {
            return mCallbacks.controllerBehaviorCallback(mCallbacks.controllerBehaviorUserData, &controller);
        }
        return static_cast<PxControllerBehaviorFlags>(0);
    }

    PxControllerBehaviorFlags getBehaviorFlags(const PxObstacle& obstacle) override {
        if (mCallbacks.obstacleBehaviorCallback) {
            return mCallbacks.obstacleBehaviorCallback(mCallbacks.obstacleBehaviorUserData, &obstacle);
        }
        return PxControllerBehaviorFlag::eCCT_CAN_RIDE_ON_OBJECT;
    }

    ControllerBehaviorCallbackInfo mCallbacks;
};

extern "C" {
    using OnContactModify = void (*)(void* data, const PxContactModifyPair* pairs, PxU32 count);
}

struct ContactModifyCallbackInfo {
    OnContactModify onContactModify = nullptr;
    void* onContactModifyUserData = nullptr;
};

class ContactModifyCallback : public PxContactModifyCallback {
public:
    ContactModifyCallback(const ContactModifyCallbackInfo *callbacks) : mCallbacks(*callbacks) {}

    void onContactModify(PxContactModifyPair* const pairs, PxU32 count) override {
        if (mCallbacks.onContactModify) {
            mCallbacks.onContactModify(mCallbacks.onContactModifyUserData, pairs, count);
        }
    }

    ContactModifyCallbackInfo mCallbacks;
};

class CCDContactModifyCallback : public PxCCDContactModifyCallback {
public:
    CCDContactModifyCallback(const ContactModifyCallbackInfo *callbacks) : mCallbacks(*callbacks) {}

    void onCCDContactModify(PxContactModifyPair* const pairs, PxU32 count) override {
        if (mCallbacks.onContactModify) {
            mCallbacks.onContactModify(mCallbacks.onContactModifyUserData, pairs, count);
        }
    }

    ContactModifyCallbackInfo mCallbacks;
};

extern "C"
{
    DLLEXPORT PxFoundation *physx_create_foundation()
    {
        return PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
    }

    DLLEXPORT PxFoundation *physx_create_foundation_with_alloc(PxAllocatorCallback *allocator)
    {
        return PxCreateFoundation(PX_PHYSICS_VERSION, *allocator, gErrorCallback);
    }

    // fixme[tolsson]: this might be iffy on Windows with DLLs if we have multiple packages
    // linking against the raw interface
    DLLEXPORT PxAllocatorCallback* get_default_allocator()
    {
        return &gAllocator;
    }

    // fixme[tolsson]: this might be iffy on Windows with DLLs if we have multiple packages
    // linking against the raw interface
    DLLEXPORT PxErrorCallback* get_default_error_callback()
    {
        return &gErrorCallback;
    }

    DLLEXPORT PxPhysics *physx_create_physics(PxFoundation *foundation)
    {
        return PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, PxTolerancesScale(), true, nullptr, nullptr);
    }

    DLLEXPORT PxQueryFilterCallback *create_raycast_filter_callback(PxRigidActor *actor_to_ignore)
    {
        RaycastFilterCallback* filter = new RaycastFilterCallback(actor_to_ignore);
        return static_cast<PxQueryFilterCallback*>(filter);
    }

    DLLEXPORT void destroy_raycast_filter_callback(PxQueryFilterCallback *callback)
    {
        RaycastFilterCallback *filter = static_cast<RaycastFilterCallback *>(callback);
        delete filter;
    }

    DLLEXPORT PxQueryFilterCallback *create_raycast_filter_callback_func(RaycastHitCallback callback, void *userData)
    {
        RaycastFilterTrampoline* filter = new RaycastFilterTrampoline(callback, userData);
        return static_cast<PxQueryFilterCallback*>(filter);
    }

    DLLEXPORT void destroy_raycast_filter_callback_func(PxQueryFilterCallback *callback)
    {
        RaycastFilterTrampoline *filter = static_cast<RaycastFilterTrampoline *>(callback);
        delete filter;
    }

    DLLEXPORT PxQueryFilterCallback *create_pre_and_post_raycast_filter_callback_func(RaycastHitCallback preFilter, PostFilterCallback postFilter, void *userData)
    {
        RaycastFilterPrePostTrampoline* filter = new RaycastFilterPrePostTrampoline(preFilter, postFilter, userData);
        return static_cast<PxQueryFilterCallback*>(filter);
    }

    DLLEXPORT void destroy_pre_and_post_raycast_filter_callback_func(PxQueryFilterCallback *callback)
    {
        RaycastFilterPrePostTrampoline *filter = static_cast<RaycastFilterPrePostTrampoline *>(callback);
        delete filter;
    }

    DLLEXPORT PxRaycastCallback *create_raycast_buffer()
    {
        return new PxRaycastBuffer;
    }

    DLLEXPORT PxSweepCallback *create_sweep_buffer()
    {
        return new PxSweepBuffer;
    }

    DLLEXPORT PxOverlapCallback *create_overlap_buffer()
    {
        return new PxOverlapBuffer;
    }

    DLLEXPORT PxRaycastCallback *create_raycast_callback(
        RaycastHitProcessTouchesCallback process_touches_callback,
        HitFinalizeQueryCallback finalize_query_callback,
        PxRaycastHit *touchesBuffer,
        PxU32 numTouches,
        void *userdata
    ) {
        return new RaycastHitCallbackTrampoline(
            process_touches_callback, finalize_query_callback, touchesBuffer, numTouches, userdata);
    }

    DLLEXPORT void delete_raycast_callback(PxRaycastCallback *callback)
    {
        delete callback;
    }

    DLLEXPORT void delete_sweep_callback(PxSweepCallback *callback)
    {
        delete callback;
    }

    DLLEXPORT void delete_overlap_callback(PxOverlapCallback *callback)
    {
        delete callback;
    }

    DLLEXPORT PxSweepCallback *create_sweep_callback(
        SweepHitProcessTouchesCallback process_touches_callback,
        HitFinalizeQueryCallback finalize_query_callback,
        PxSweepHit *touchesBuffer,
        PxU32 numTouches,
        void *userdata
    ) {
        return new SweepHitCallbackTrampoline(
            process_touches_callback, finalize_query_callback, touchesBuffer, numTouches, userdata
        );
    }

    DLLEXPORT PxOverlapCallback *create_overlap_callback(
        OverlapHitProcessTouchesCallback process_touches_callback,
        HitFinalizeQueryCallback finalize_query_callback,
        PxOverlapHit *touchesBuffer,
        PxU32 numTouches,
        void *userdata
    ) {
        return new OverlapHitCallbackTrampoline(
            process_touches_callback, finalize_query_callback, touchesBuffer, numTouches, userdata
        );
    }

    DLLEXPORT PxAllocatorCallback *create_alloc_callback(
        AllocCallback alloc_callback,
        DeallocCallback dealloc_callback,
        void *userdata
    ) {
        return new CustomAllocatorTrampoline(alloc_callback, dealloc_callback, userdata);
    }

    DLLEXPORT void *get_alloc_callback_user_data(PxAllocatorCallback *allocator) {
        CustomAllocatorTrampoline *trampoline = static_cast<CustomAllocatorTrampoline *>(allocator);
        return trampoline->mUserData;
    }

    DLLEXPORT PxProfilerCallback *create_profiler_callback(
        ZoneStartCallback zone_start_callback,
        ZoneEndCallback zone_end_callback,
        void *userdata
    ) {
        return new CustomProfilerTrampoline(zone_start_callback, zone_end_callback, userdata);
    }

    DLLEXPORT PxErrorCallback *create_error_callback(
        ErrorCallback error_callback,
        void* userdata
    ) {
        return new ErrorTrampoline(error_callback, userdata);
    }


    DLLEXPORT PxAssertHandler *create_assert_handler(
        AssertHandler on_assert,
        void* userdata
    ) {
        return new AssertTrampoline(on_assert, userdata);
    }

    // simulation event

    DLLEXPORT PxSimulationEventCallback *create_simulation_event_callbacks(const SimulationEventCallbackInfo *callbacks)
    {
        SimulationEventTrampoline *trampoline = new SimulationEventTrampoline(callbacks);
        return static_cast<PxSimulationEventCallback *>(trampoline);
    }

    DLLEXPORT SimulationEventCallbackInfo *get_simulation_event_info(PxSimulationEventCallback *callback)
    {
        SimulationEventTrampoline *trampoline = static_cast<SimulationEventTrampoline *>(callback);
        return &trampoline->mCallbacks;
    }

    DLLEXPORT void destroy_simulation_event_callbacks(PxSimulationEventCallback *callback)
    {
        SimulationEventTrampoline *trampoline = static_cast<SimulationEventTrampoline *>(callback);
        delete trampoline;
    }

    // hit report

    DLLEXPORT PxUserControllerHitReport *create_user_controller_hit_report(const UserControllerHitReportInfo *callbacks)
    {
        UserControllerHitReport *report = new UserControllerHitReport(callbacks);
        return static_cast<PxUserControllerHitReport *>(report);
    }

    DLLEXPORT UserControllerHitReportInfo *get_user_controller_hit_info(PxUserControllerHitReport *callback)
    {
        UserControllerHitReport *report = static_cast<UserControllerHitReport *>(callback);
        return &report->mCallbacks;
    }

    DLLEXPORT void destroy_user_controller_hit_report(PxUserControllerHitReport *callback)
    {
        UserControllerHitReport *report = static_cast<UserControllerHitReport *>(callback);
        delete report;
    }

    // controller behavior

    DLLEXPORT PxControllerBehaviorCallback *create_controller_behavior_callbacks(const ControllerBehaviorCallbackInfo *callbacks)
    {
        ControllerBehaviorCallback *behavior = new ControllerBehaviorCallback(callbacks);
        return static_cast<PxControllerBehaviorCallback *>(behavior);
    }

    DLLEXPORT ControllerBehaviorCallbackInfo *get_controller_behavior_info(PxControllerBehaviorCallback *callback)
    {
        ControllerBehaviorCallback *behavior = static_cast<ControllerBehaviorCallback *>(callback);
        return &behavior->mCallbacks;
    }

    DLLEXPORT void destroy_controller_behavior_callbacks(PxControllerBehaviorCallback *callback)
    {
        ControllerBehaviorCallback *behavior = static_cast<ControllerBehaviorCallback *>(callback);
        delete behavior;
    }

    // contact modify

    DLLEXPORT PxContactModifyCallback *create_contact_modify_callbacks(const ContactModifyCallbackInfo *callbacks)
    {
        ContactModifyCallback *modify = new ContactModifyCallback(callbacks);
        return static_cast<PxContactModifyCallback *>(modify);
    }

    DLLEXPORT ContactModifyCallbackInfo *get_contact_modify_info(PxContactModifyCallback *callback)
    {
        ContactModifyCallback *modify = static_cast<ContactModifyCallback *>(callback);
        return &modify->mCallbacks;
    }

    DLLEXPORT void destroy_contact_modify_callbacks(PxContactModifyCallback *callback)
    {
        ContactModifyCallback *modify = static_cast<ContactModifyCallback *>(callback);
        delete modify;
    }

    // ccd contact modify

    DLLEXPORT PxCCDContactModifyCallback *create_ccd_contact_modify_callbacks(const ContactModifyCallbackInfo *callbacks)
    {
        CCDContactModifyCallback *modify = new CCDContactModifyCallback(callbacks);
        return static_cast<PxCCDContactModifyCallback *>(modify);
    }

    DLLEXPORT ContactModifyCallbackInfo *get_ccd_contact_modify_info(PxCCDContactModifyCallback *callback)
    {
        CCDContactModifyCallback *modify = static_cast<CCDContactModifyCallback *>(callback);
        return &modify->mCallbacks;
    }

    DLLEXPORT void destroy_ccd_contact_modify_callbacks(PxCCDContactModifyCallback *callback)
    {
        CCDContactModifyCallback *modify = static_cast<CCDContactModifyCallback *>(callback);
        delete modify;
    }

    // filter shader

    DLLEXPORT SimulationFilterShader get_default_simulation_filter_shader()
    {
        return DefaultSimulationFilterShader;
    }

    DLLEXPORT void set_default_filter_shader(PxSceneDesc* desc)
    {
        desc->filterShader = PxDefaultSimulationFilterShader;
        desc->filterShaderData = nullptr;
        desc->filterShaderDataSize = 0;
    }

    DLLEXPORT FilterShaderHandle* create_custom_filter_shader(SimulationFilterShader filter)
    {
        return new FilterShaderHandle{filter};
    }

    DLLEXPORT void destroy_custom_filter_shader(FilterShaderHandle* filterHandle) {
        delete filterHandle;
    }

    DLLEXPORT void set_custom_filter_shader(PxSceneDesc* desc, FilterShaderHandle* filterHandle)
    {
        desc->filterShader = FilterShaderTrampoline;
        desc->filterShaderData = filterHandle;
        desc->filterShaderDataSize = sizeof(FilterShaderHandle);
    }

    DLLEXPORT void set_custom_filter_shader_with_default(PxSceneDesc* desc, FilterShaderHandle* filterHandle)
    {
        desc->filterShader = FilterShaderTrampolineWithDefault;
        desc->filterShaderData = filterHandle;
        desc->filterShaderDataSize = sizeof(FilterShaderHandle);
    }

	// Not generated, used only for testing and examples!
    DLLEXPORT void PxAssertHandler_opCall_mut(physx_PxErrorCallback* self__pod, char const* expr, char const* file, int32_t line, bool* ignore ) {
		physx::PxAssertHandler* self_ = reinterpret_cast<physx::PxAssertHandler*>(self__pod);
		(*self_)(expr, file, line, *ignore);
	};
}
