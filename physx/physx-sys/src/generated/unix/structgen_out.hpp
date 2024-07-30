struct physx_PxAllocatorCallback;
struct physx_PxErrorCallback;
struct physx_PxAssertHandler;
struct physx_PxInputStream;
struct physx_PxInputData;
struct physx_PxOutputStream;
struct physx_PxVec2;
struct physx_PxVec3;
struct physx_PxVec4;
struct physx_PxQuat;
struct physx_PxMat33;
struct physx_PxMat34;
struct physx_PxMat44;
struct physx_PxTransform;
struct physx_PxPlane;
struct physx_PxBounds3;
struct physx_PxAllocatorCallback {
    void* vtable_;
};
struct physx_PxAssertHandler {
    void* vtable_;
};
struct physx_PxAllocationListener;
struct physx_PxFoundation {
    void* vtable_;
};
struct physx_PxProfilerCallback;
struct physx_PxAllocator {
    char structgen_pad0[1];
};
struct physx_PxRawAllocator {
    char structgen_pad0[1];
};
struct physx_PxVirtualAllocatorCallback {
    void* vtable_;
};
struct physx_PxVirtualAllocator {
    char structgen_pad0[16];
};
struct physx_PxUserAllocated {
    char structgen_pad0[1];
};
union physx_PxTempAllocatorChunk {
    physx_PxTempAllocatorChunk* mNext;
    uint32_t mIndex;
    uint8_t mPad[16];
};
struct physx_PxTempAllocator {
    char structgen_pad0[1];
};
struct physx_PxLogTwo;
struct physx_PxUnConst;
struct physx_PxBitAndByte {
    char structgen_pad0[1];
};
struct physx_PxBitMap {
    char structgen_pad0[16];
};
struct physx_PxVec3 {
    float x;
    float y;
    float z;
};
struct physx_PxVec3Padded {
    float x;
    float y;
    float z;
    uint32_t padding;
};
struct physx_PxQuat {
    float x;
    float y;
    float z;
    float w;
};
struct physx_PxTransform {
    physx_PxQuat q;
    physx_PxVec3 p;
};
struct physx_PxTransformPadded {
    physx_PxTransform transform;
    uint32_t padding;
};
struct physx_PxMat33 {
    physx_PxVec3 column0;
    physx_PxVec3 column1;
    physx_PxVec3 column2;
};
struct physx_PxBounds3 {
    physx_PxVec3 minimum;
    physx_PxVec3 maximum;
};
struct physx_PxErrorCallback {
    void* vtable_;
};
struct physx_PxAllocationListener {
    void* vtable_;
};
struct physx_PxBroadcastingAllocator {
    char structgen_pad0[176];
};
struct physx_PxBroadcastingErrorCallback {
    char structgen_pad0[160];
};
struct physx_PxHash;
struct physx_PxInputStream {
    void* vtable_;
};
struct physx_PxInputData {
    void* vtable_;
};
struct physx_PxOutputStream {
    void* vtable_;
};
struct physx_PxVec4 {
    float x;
    float y;
    float z;
    float w;
};
struct physx_PxMat44 {
    physx_PxVec4 column0;
    physx_PxVec4 column1;
    physx_PxVec4 column2;
    physx_PxVec4 column3;
};
struct physx_PxPlane {
    physx_PxVec3 n;
    float d;
};
struct physx_Interpolation {
    char structgen_pad0[1];
};
struct physx_PxMutexImpl {
    char structgen_pad0[1];
};
struct physx_PxReadWriteLock {
    char structgen_pad0[8];
};
struct physx_PxProfilerCallback {
    void* vtable_;
};
struct physx_PxProfileScoped {
    physx_PxProfilerCallback* mCallback;
    char const* mEventName;
    void* mProfilerData;
    uint64_t mContextId;
    bool mDetached;
    char structgen_pad0[7];
};
struct physx_PxSListEntry {
    char structgen_pad0[16];
};
struct physx_PxSListImpl {
    char structgen_pad0[1];
};
struct physx_PxSyncImpl {
    char structgen_pad0[1];
};
struct physx_PxRunnable {
    void* vtable_;
};
struct physx_PxCounterFrequencyToTensOfNanos {
    uint64_t mNumerator;
    uint64_t mDenominator;
};
struct physx_PxTime {
    char structgen_pad0[8];
};
struct physx_PxVec2 {
    float x;
    float y;
};
struct physx_PxStridedData {
    uint32_t stride;
    char structgen_pad0[4];
    void const* data;
};
struct physx_PxBoundedData {
    uint32_t stride;
    char structgen_pad0[4];
    void const* data;
    uint32_t count;
    char structgen_pad1[4];
};
struct physx_PxDebugPoint {
    physx_PxVec3 pos;
    uint32_t color;
};
struct physx_PxDebugLine {
    physx_PxVec3 pos0;
    uint32_t color0;
    physx_PxVec3 pos1;
    uint32_t color1;
};
struct physx_PxDebugTriangle {
    physx_PxVec3 pos0;
    uint32_t color0;
    physx_PxVec3 pos1;
    uint32_t color1;
    physx_PxVec3 pos2;
    uint32_t color2;
};
struct physx_PxDebugText {
    physx_PxVec3 position;
    float size;
    uint32_t color;
    char structgen_pad0[4];
    char const* string;
};
struct physx_PxRenderBuffer {
    void* vtable_;
};
struct physx_PxBase;
struct physx_PxSerializationContext;
struct physx_PxRepXSerializer;
struct physx_PxSerializer;
struct physx_PxPhysics;
struct physx_PxCollection;
struct physx_PxProcessPxBaseCallback {
    void* vtable_;
};
struct physx_PxSerializationContext {
    void* vtable_;
};
struct physx_PxDeserializationContext {
    char structgen_pad0[16];
};
struct physx_PxSerializationRegistry {
    void* vtable_;
};
struct physx_PxCollection {
    void* vtable_;
};
struct physx_PxTypeInfo;
struct physx_PxMaterial;
struct physx_PxFEMSoftBodyMaterial;
struct physx_PxFEMClothMaterial;
struct physx_PxPBDMaterial;
struct physx_PxFLIPMaterial;
struct physx_PxMPMMaterial;
struct physx_PxCustomMaterial;
struct physx_PxConvexMesh;
struct physx_PxTriangleMesh;
struct physx_PxBVH33TriangleMesh;
struct physx_PxBVH34TriangleMesh;
struct physx_PxTetrahedronMesh;
struct physx_PxHeightField;
struct physx_PxActor;
struct physx_PxRigidActor;
struct physx_PxRigidBody;
struct physx_PxRigidDynamic;
struct physx_PxRigidStatic;
struct physx_PxArticulationLink;
struct physx_PxArticulationJointReducedCoordinate;
struct physx_PxArticulationReducedCoordinate;
struct physx_PxAggregate;
struct physx_PxConstraint;
struct physx_PxShape;
struct physx_PxPruningStructure;
struct physx_PxParticleSystem;
struct physx_PxPBDParticleSystem;
struct physx_PxFLIPParticleSystem;
struct physx_PxMPMParticleSystem;
struct physx_PxCustomParticleSystem;
struct physx_PxSoftBody;
struct physx_PxFEMCloth;
struct physx_PxHairSystem;
struct physx_PxParticleBuffer;
struct physx_PxParticleAndDiffuseBuffer;
struct physx_PxParticleClothBuffer;
struct physx_PxParticleRigidBuffer;
struct physx_PxBase {
    char structgen_pad0[16];
};
struct physx_PxRefCounted {
    char structgen_pad0[16];
};
struct physx_PxTolerancesScale {
    float length;
    float speed;
};
struct physx_PxStringTable {
    void* vtable_;
};
struct physx_PxSerializer {
    void* vtable_;
};
struct physx_PxMetaDataEntry {
    char const* type;
    char const* name;
    uint32_t offset;
    uint32_t size;
    uint32_t count;
    uint32_t offsetSize;
    uint32_t flags;
    uint32_t alignment;
};
struct physx_PxInsertionCallback {
    void* vtable_;
};
struct physx_PxBaseTask;
struct physx_PxTask;
struct physx_PxLightCpuTask;
struct physx_PxCpuDispatcher;
struct physx_PxTaskManager {
    void* vtable_;
};
struct physx_PxCpuDispatcher {
    void* vtable_;
};
struct physx_PxBaseTask {
    char structgen_pad0[24];
};
struct physx_PxTask {
    char structgen_pad0[32];
};
struct physx_PxLightCpuTask {
    char structgen_pad0[40];
};
struct physx_PxGeometry {
    char structgen_pad0[4];
    float mTypePadding;
};
struct physx_PxBoxGeometry {
    char structgen_pad0[4];
    float mTypePadding;
    physx_PxVec3 halfExtents;
};
struct physx_PxBVHRaycastCallback {
    void* vtable_;
};
struct physx_PxBVHOverlapCallback {
    void* vtable_;
};
struct physx_PxBVHTraversalCallback {
    void* vtable_;
};
struct physx_PxBVH {
    char structgen_pad0[16];
};
struct physx_PxGeomIndexPair;
struct physx_PxCapsuleGeometry {
    char structgen_pad0[4];
    float mTypePadding;
    float radius;
    float halfHeight;
};
struct physx_PxHullPolygon {
    float mPlane[4];
    uint16_t mNbVerts;
    uint16_t mIndexBase;
};
struct physx_PxConvexMesh {
    char structgen_pad0[16];
};
struct physx_PxMeshScale {
    physx_PxVec3 scale;
    physx_PxQuat rotation;
};
struct physx_PxConvexMeshGeometry {
    char structgen_pad0[4];
    float mTypePadding;
    physx_PxMeshScale scale;
    char structgen_pad1[4];
    physx_PxConvexMesh* convexMesh;
    uint8_t meshFlags;
    char structgen_pad2[7];
};
struct physx_PxSphereGeometry {
    char structgen_pad0[4];
    float mTypePadding;
    float radius;
};
struct physx_PxPlaneGeometry {
    char structgen_pad0[4];
    float mTypePadding;
};
struct physx_PxTriangleMeshGeometry {
    char structgen_pad0[4];
    float mTypePadding;
    physx_PxMeshScale scale;
    uint8_t meshFlags;
    char structgen_pad1[3];
    physx_PxTriangleMesh* triangleMesh;
};
struct physx_PxHeightFieldGeometry {
    char structgen_pad0[4];
    float mTypePadding;
    physx_PxHeightField* heightField;
    float heightScale;
    float rowScale;
    float columnScale;
    uint8_t heightFieldFlags;
    char structgen_pad1[3];
};
struct physx_PxParticleSystemGeometry {
    char structgen_pad0[4];
    float mTypePadding;
    int32_t mSolverType;
};
struct physx_PxHairSystemGeometry {
    char structgen_pad0[4];
    float mTypePadding;
};
struct physx_PxTetrahedronMeshGeometry {
    char structgen_pad0[4];
    float mTypePadding;
    physx_PxTetrahedronMesh* tetrahedronMesh;
};
struct physx_PxQueryHit {
    uint32_t faceIndex;
};
struct physx_PxLocationHit {
    uint32_t faceIndex;
    uint16_t flags;
    char structgen_pad0[2];
    physx_PxVec3 position;
    physx_PxVec3 normal;
    float distance;
};
struct physx_PxGeomRaycastHit {
    uint32_t faceIndex;
    uint16_t flags;
    char structgen_pad0[2];
    physx_PxVec3 position;
    physx_PxVec3 normal;
    float distance;
    float u;
    float v;
};
struct physx_PxGeomOverlapHit {
    uint32_t faceIndex;
};
struct physx_PxGeomSweepHit {
    uint32_t faceIndex;
    uint16_t flags;
    char structgen_pad0[2];
    physx_PxVec3 position;
    physx_PxVec3 normal;
    float distance;
};
struct physx_PxGeomIndexPair {
    uint32_t id0;
    uint32_t id1;
};
struct physx_PxQueryThreadContext {
    char structgen_pad0[1];
};
struct physx_PxContactBuffer;
struct physx_PxRenderOutput;
struct physx_PxMassProperties;
struct physx_PxCustomGeometryType {
    char structgen_pad0[4];
};
struct physx_PxCustomGeometryCallbacks {
    void* vtable_;
};
struct physx_PxCustomGeometry {
    char structgen_pad0[4];
    float mTypePadding;
    physx_PxCustomGeometryCallbacks* callbacks;
};
struct physx_PxGeometryHolder {
    char structgen_pad0[56];
};
struct physx_PxGeometryQuery {
    char structgen_pad0[1];
};
struct physx_PxHeightFieldSample {
    int16_t height;
    physx_PxBitAndByte materialIndex0;
    physx_PxBitAndByte materialIndex1;
};
struct physx_PxHeightFieldDesc;
struct physx_PxHeightField {
    char structgen_pad0[16];
};
struct physx_PxHeightFieldDesc {
    uint32_t nbRows;
    uint32_t nbColumns;
    int32_t format;
    char structgen_pad0[4];
    physx_PxStridedData samples;
    float convexEdgeThreshold;
    uint16_t flags;
    char structgen_pad1[2];
};
struct physx_PxTriangle;
struct physx_PxMeshQuery {
    char structgen_pad0[1];
};
struct physx_PxSimpleTriangleMesh {
    physx_PxBoundedData points;
    physx_PxBoundedData triangles;
    uint16_t flags;
    char structgen_pad0[6];
};
struct physx_PxTriangle {
    physx_PxVec3 verts[3];
};
struct physx_PxTrianglePadded {
    physx_PxVec3 verts[3];
    uint32_t padding;
};
struct physx_PxTriangleMesh {
    char structgen_pad0[16];
};
struct physx_PxBVH34TriangleMesh {
    char structgen_pad0[16];
};
struct physx_PxTetrahedron {
    physx_PxVec3 verts[4];
};
struct physx_PxSoftBodyAuxData {
    char structgen_pad0[16];
};
struct physx_PxTetrahedronMesh {
    char structgen_pad0[16];
};
struct physx_PxSoftBodyMesh {
    char structgen_pad0[16];
};
struct physx_PxCollisionMeshMappingData {
    char structgen_pad0[8];
};
struct physx_PxSoftBodyCollisionData {
    char structgen_pad0[1];
};
struct physx_PxTetrahedronMeshData {
    char structgen_pad0[1];
};
struct physx_PxSoftBodySimulationData {
    char structgen_pad0[1];
};
struct physx_PxCollisionTetrahedronMeshData {
    char structgen_pad0[8];
};
struct physx_PxSimulationTetrahedronMeshData {
    char structgen_pad0[8];
};
struct physx_PxScene;
struct physx_PxActor {
    char structgen_pad0[16];
    void* userData;
};
struct physx_PxAggregate {
    char structgen_pad0[16];
    void* userData;
};
struct physx_PxSpringModifiers {
    float stiffness;
    float damping;
    char structgen_pad0[8];
};
struct physx_PxRestitutionModifiers {
    float restitution;
    float velocityThreshold;
    char structgen_pad0[8];
};
union physx_Px1DConstraintMods {
    physx_PxSpringModifiers spring;
    physx_PxRestitutionModifiers bounce;
};
struct physx_Px1DConstraint {
    physx_PxVec3 linear0;
    float geometricError;
    physx_PxVec3 angular0;
    float velocityTarget;
    physx_PxVec3 linear1;
    float minImpulse;
    physx_PxVec3 angular1;
    float maxImpulse;
    physx_Px1DConstraintMods mods;
    float forInternalUse;
    uint16_t flags;
    uint16_t solveHint;
    char structgen_pad0[8];
};
struct physx_PxConstraintInvMassScale {
    float linear0;
    float angular0;
    float linear1;
    float angular1;
};
struct physx_PxConstraintVisualizer {
    void* vtable_;
};
struct physx_PxConstraintConnector {
    void* vtable_;
};
struct physx_PxContactPoint {
    physx_PxVec3 normal;
    float separation;
    physx_PxVec3 point;
    float maxImpulse;
    physx_PxVec3 targetVel;
    float staticFriction;
    uint8_t materialFlags;
    char structgen_pad0[3];
    uint32_t internalFaceIndex1;
    float dynamicFriction;
    float restitution;
    float damping;
    char structgen_pad1[12];
};
struct physx_PxTGSSolverBodyVel;
struct physx_PxSolverBody {
    physx_PxVec3 linearVelocity;
    uint16_t maxSolverNormalProgress;
    uint16_t maxSolverFrictionProgress;
    physx_PxVec3 angularState;
    uint32_t solverProgress;
};
struct physx_PxSolverBodyData {
    physx_PxVec3 linearVelocity;
    float invMass;
    physx_PxVec3 angularVelocity;
    float reportThreshold;
    physx_PxMat33 sqrtInvInertia;
    float penBiasClamp;
    uint32_t nodeIndex;
    float maxContactImpulse;
    physx_PxTransform body2World;
    uint16_t pad;
    char structgen_pad0[2];
};
struct physx_PxConstraintBatchHeader {
    uint32_t startIndex;
    uint16_t stride;
    uint16_t constraintType;
};
struct physx_PxSolverConstraintDesc {
    char structgen_pad0[16];
    uint32_t bodyADataIndex;
    uint32_t bodyBDataIndex;
    uint32_t linkIndexA;
    uint32_t linkIndexB;
    uint8_t* constraint;
    void* writeBack;
    uint16_t progressA;
    uint16_t progressB;
    uint16_t constraintLengthOver16;
    uint8_t padding[10];
};
struct physx_PxSolverConstraintPrepDescBase {
    physx_PxConstraintInvMassScale invMassScales;
    physx_PxSolverConstraintDesc* desc;
    physx_PxSolverBody const* body0;
    physx_PxSolverBody const* body1;
    physx_PxSolverBodyData const* data0;
    physx_PxSolverBodyData const* data1;
    physx_PxTransform bodyFrame0;
    physx_PxTransform bodyFrame1;
    int32_t bodyState0;
    int32_t bodyState1;
    char structgen_pad0[8];
};
struct physx_PxSolverConstraintPrepDesc {
    physx_PxConstraintInvMassScale invMassScales;
    physx_PxSolverConstraintDesc* desc;
    physx_PxSolverBody const* body0;
    physx_PxSolverBody const* body1;
    physx_PxSolverBodyData const* data0;
    physx_PxSolverBodyData const* data1;
    physx_PxTransform bodyFrame0;
    physx_PxTransform bodyFrame1;
    int32_t bodyState0;
    int32_t bodyState1;
    char structgen_pad0[8];
    physx_Px1DConstraint* rows;
    uint32_t numRows;
    float linBreakForce;
    float angBreakForce;
    float minResponseThreshold;
    void* writeback;
    bool disablePreprocessing;
    bool improvedSlerp;
    bool driveLimitsAreForces;
    bool extendedLimits;
    bool disableConstraint;
    char structgen_pad1[3];
    physx_PxVec3Padded body0WorldOffset;
    char structgen_pad2[8];
};
struct physx_PxSolverContactDesc {
    physx_PxConstraintInvMassScale invMassScales;
    physx_PxSolverConstraintDesc* desc;
    physx_PxSolverBody const* body0;
    physx_PxSolverBody const* body1;
    physx_PxSolverBodyData const* data0;
    physx_PxSolverBodyData const* data1;
    physx_PxTransform bodyFrame0;
    physx_PxTransform bodyFrame1;
    int32_t bodyState0;
    int32_t bodyState1;
    void* shapeInteraction;
    physx_PxContactPoint* contacts;
    uint32_t numContacts;
    bool hasMaxImpulse;
    bool disableStrongFriction;
    bool hasForceThresholds;
    char structgen_pad0[1];
    float restDistance;
    float maxCCDSeparation;
    uint8_t* frictionPtr;
    uint8_t frictionCount;
    char structgen_pad1[7];
    float* contactForces;
    uint32_t startFrictionPatchIndex;
    uint32_t numFrictionPatches;
    uint32_t startContactPatchIndex;
    uint16_t numContactPatches;
    uint16_t axisConstraintCount;
    float offsetSlop;
    char structgen_pad2[12];
};
struct physx_PxConstraintAllocator {
    void* vtable_;
};
struct physx_PxArticulationLimit {
    float low;
    float high;
};
struct physx_PxArticulationDrive {
    float stiffness;
    float damping;
    float maxForce;
    int32_t driveType;
};
struct physx_PxTGSSolverBodyVel {
    physx_PxVec3 linearVelocity;
    uint16_t nbStaticInteractions;
    uint16_t maxDynamicPartition;
    physx_PxVec3 angularVelocity;
    uint32_t partitionMask;
    physx_PxVec3 deltaAngDt;
    float maxAngVel;
    physx_PxVec3 deltaLinDt;
    uint16_t lockFlags;
    bool isKinematic;
    uint8_t pad;
};
struct physx_PxTGSSolverBodyTxInertia {
    physx_PxTransform deltaBody2World;
    physx_PxMat33 sqrtInvInertia;
};
struct physx_PxTGSSolverBodyData {
    physx_PxVec3 originalLinearVelocity;
    float maxContactImpulse;
    physx_PxVec3 originalAngularVelocity;
    float penBiasClamp;
    float invMass;
    uint32_t nodeIndex;
    float reportThreshold;
    uint32_t pad;
};
struct physx_PxTGSSolverConstraintPrepDescBase {
    physx_PxConstraintInvMassScale invMassScales;
    physx_PxSolverConstraintDesc* desc;
    physx_PxTGSSolverBodyVel const* body0;
    physx_PxTGSSolverBodyVel const* body1;
    physx_PxTGSSolverBodyTxInertia const* body0TxI;
    physx_PxTGSSolverBodyTxInertia const* body1TxI;
    physx_PxTGSSolverBodyData const* bodyData0;
    physx_PxTGSSolverBodyData const* bodyData1;
    physx_PxTransform bodyFrame0;
    physx_PxTransform bodyFrame1;
    int32_t bodyState0;
    int32_t bodyState1;
    char structgen_pad0[8];
};
struct physx_PxTGSSolverConstraintPrepDesc {
    physx_PxConstraintInvMassScale invMassScales;
    physx_PxSolverConstraintDesc* desc;
    physx_PxTGSSolverBodyVel const* body0;
    physx_PxTGSSolverBodyVel const* body1;
    physx_PxTGSSolverBodyTxInertia const* body0TxI;
    physx_PxTGSSolverBodyTxInertia const* body1TxI;
    physx_PxTGSSolverBodyData const* bodyData0;
    physx_PxTGSSolverBodyData const* bodyData1;
    physx_PxTransform bodyFrame0;
    physx_PxTransform bodyFrame1;
    int32_t bodyState0;
    int32_t bodyState1;
    physx_Px1DConstraint* rows;
    uint32_t numRows;
    float linBreakForce;
    float angBreakForce;
    float minResponseThreshold;
    void* writeback;
    bool disablePreprocessing;
    bool improvedSlerp;
    bool driveLimitsAreForces;
    bool extendedLimits;
    bool disableConstraint;
    char structgen_pad0[3];
    physx_PxVec3Padded body0WorldOffset;
    physx_PxVec3Padded cA2w;
    physx_PxVec3Padded cB2w;
};
struct physx_PxTGSSolverContactDesc {
    physx_PxConstraintInvMassScale invMassScales;
    physx_PxSolverConstraintDesc* desc;
    physx_PxTGSSolverBodyVel const* body0;
    physx_PxTGSSolverBodyVel const* body1;
    physx_PxTGSSolverBodyTxInertia const* body0TxI;
    physx_PxTGSSolverBodyTxInertia const* body1TxI;
    physx_PxTGSSolverBodyData const* bodyData0;
    physx_PxTGSSolverBodyData const* bodyData1;
    physx_PxTransform bodyFrame0;
    physx_PxTransform bodyFrame1;
    int32_t bodyState0;
    int32_t bodyState1;
    void* shapeInteraction;
    physx_PxContactPoint* contacts;
    uint32_t numContacts;
    bool hasMaxImpulse;
    bool disableStrongFriction;
    bool hasForceThresholds;
    char structgen_pad0[1];
    float restDistance;
    float maxCCDSeparation;
    uint8_t* frictionPtr;
    uint8_t frictionCount;
    char structgen_pad1[7];
    float* contactForces;
    uint32_t startFrictionPatchIndex;
    uint32_t numFrictionPatches;
    uint32_t startContactPatchIndex;
    uint16_t numContactPatches;
    uint16_t axisConstraintCount;
    float maxImpulse;
    float torsionalPatchRadius;
    float minTorsionalPatchRadius;
    float offsetSlop;
};
struct physx_PxArticulationSpatialTendon;
struct physx_PxArticulationFixedTendon;
struct physx_PxArticulationTendonLimit {
    float lowLimit;
    float highLimit;
};
struct physx_PxArticulationAttachment {
    char structgen_pad0[16];
    void* userData;
};
struct physx_PxArticulationTendonJoint {
    char structgen_pad0[16];
    void* userData;
};
struct physx_PxArticulationTendon {
    char structgen_pad0[16];
    void* userData;
};
struct physx_PxArticulationSpatialTendon {
    char structgen_pad0[16];
    void* userData;
};
struct physx_PxArticulationFixedTendon {
    char structgen_pad0[16];
    void* userData;
};
struct physx_PxSpatialForce {
    physx_PxVec3 force;
    float pad0;
    physx_PxVec3 torque;
    float pad1;
};
struct physx_PxSpatialVelocity {
    physx_PxVec3 linear;
    float pad0;
    physx_PxVec3 angular;
    float pad1;
};
struct physx_PxArticulationRootLinkData {
    physx_PxTransform transform;
    physx_PxVec3 worldLinVel;
    physx_PxVec3 worldAngVel;
    physx_PxVec3 worldLinAccel;
    physx_PxVec3 worldAngAccel;
};
struct physx_PxArticulationCache {
    physx_PxSpatialForce* externalForces;
    float* denseJacobian;
    float* massMatrix;
    float* jointVelocity;
    float* jointAcceleration;
    float* jointPosition;
    float* jointForce;
    float* jointSolverForces;
    physx_PxSpatialVelocity* linkVelocity;
    physx_PxSpatialVelocity* linkAcceleration;
    physx_PxArticulationRootLinkData* rootLinkData;
    physx_PxSpatialForce* sensorForces;
    float* coefficientMatrix;
    float* lambda;
    void* scratchMemory;
    void* scratchAllocator;
    uint32_t version;
    char structgen_pad0[4];
};
struct physx_PxArticulationSensor {
    char structgen_pad0[16];
    void* userData;
};
struct physx_PxArticulationReducedCoordinate {
    char structgen_pad0[16];
    void* userData;
};
struct physx_PxArticulationJointReducedCoordinate {
    char structgen_pad0[16];
    void* userData;
};
struct physx_PxFilterData;
struct physx_PxBaseMaterial;
struct physx_PxShape {
    char structgen_pad0[16];
    void* userData;
};
struct physx_PxRigidActor {
    char structgen_pad0[16];
    void* userData;
};
struct physx_PxNodeIndex {
    char structgen_pad0[8];
};
struct physx_PxRigidBody {
    char structgen_pad0[16];
    void* userData;
};
struct physx_PxArticulationLink {
    char structgen_pad0[16];
    void* userData;
};
struct physx_PxConeLimitedConstraint {
    physx_PxVec3 mAxis;
    float mAngle;
    float mLowLimit;
    float mHighLimit;
};
struct physx_PxConeLimitParams {
    physx_PxVec4 lowHighLimits;
    physx_PxVec4 axisAngle;
};
struct physx_PxConstraintShaderTable {
    void * solverPrep;
    char structgen_pad0[8];
    void * visualize;
    int32_t flag;
    char structgen_pad1[4];
};
struct physx_PxConstraint {
    char structgen_pad0[16];
    void* userData;
};
struct physx_PxMassModificationProps {
    float mInvMassScale0;
    float mInvInertiaScale0;
    float mInvMassScale1;
    float mInvInertiaScale1;
};
struct physx_PxContactPatch {
    physx_PxMassModificationProps mMassModification;
    physx_PxVec3 normal;
    float restitution;
    float dynamicFriction;
    float staticFriction;
    float damping;
    uint16_t startContactIndex;
    uint8_t nbContacts;
    uint8_t materialFlags;
    uint16_t internalFlags;
    uint16_t materialIndex0;
    uint16_t materialIndex1;
    uint16_t pad[5];
};
struct physx_PxContact {
    physx_PxVec3 contact;
    float separation;
};
struct physx_PxExtendedContact {
    physx_PxVec3 contact;
    float separation;
    physx_PxVec3 targetVelocity;
    float maxImpulse;
};
struct physx_PxModifiableContact {
    physx_PxVec3 contact;
    float separation;
    physx_PxVec3 targetVelocity;
    float maxImpulse;
    physx_PxVec3 normal;
    float restitution;
    uint32_t materialFlags;
    uint16_t materialIndex0;
    uint16_t materialIndex1;
    float staticFriction;
    float dynamicFriction;
};
struct physx_PxContactStreamIterator {
    physx_PxVec3 zero;
    char structgen_pad0[4];
    physx_PxContactPatch const* patch;
    physx_PxContact const* contact;
    uint32_t const* faceIndice;
    uint32_t totalPatches;
    uint32_t totalContacts;
    uint32_t nextContactIndex;
    uint32_t nextPatchIndex;
    uint32_t contactPatchHeaderSize;
    uint32_t contactPointSize;
    int32_t mStreamFormat;
    uint32_t forceNoResponse;
    bool pointStepped;
    char structgen_pad1[3];
    uint32_t hasFaceIndices;
};
struct physx_PxGpuContactPair {
    uint8_t* contactPatches;
    uint8_t* contactPoints;
    float* contactForces;
    uint32_t transformCacheRef0;
    uint32_t transformCacheRef1;
    physx_PxNodeIndex nodeIndex0;
    physx_PxNodeIndex nodeIndex1;
    physx_PxActor* actor0;
    physx_PxActor* actor1;
    uint16_t nbContacts;
    uint16_t nbPatches;
    char structgen_pad0[4];
};
struct physx_PxContactSet {
    char structgen_pad0[16];
};
struct physx_PxContactModifyPair {
    physx_PxRigidActor const* actor[2];
    physx_PxShape const* shape[2];
    physx_PxTransform transform[2];
    physx_PxContactSet contacts;
};
struct physx_PxContactModifyCallback {
    void* vtable_;
};
struct physx_PxCCDContactModifyCallback {
    void* vtable_;
};
struct physx_PxDeletionListener {
    void* vtable_;
};
struct physx_PxBaseMaterial {
    char structgen_pad0[16];
    void* userData;
};
struct physx_PxFEMMaterial {
    char structgen_pad0[16];
    void* userData;
};
struct physx_PxFilterData {
    uint32_t word0;
    uint32_t word1;
    uint32_t word2;
    uint32_t word3;
};
struct physx_PxSimulationFilterCallback {
    void* vtable_;
};
struct physx_PxParticleRigidFilterPair {
    uint64_t mID0;
    uint64_t mID1;
};
struct physx_PxLockedData {
    void* vtable_;
};
struct physx_PxMaterial {
    char structgen_pad0[16];
    void* userData;
};
struct physx_PxGpuParticleBufferIndexPair {
    uint32_t systemIndex;
    uint32_t bufferIndex;
};
struct physx_PxCudaContextManager;
struct physx_PxParticleRigidAttachment;
struct physx_PxParticleVolume {
    physx_PxBounds3 bound;
    uint32_t particleIndicesOffset;
    uint32_t numParticles;
};
struct physx_PxDiffuseParticleParams {
    float threshold;
    float lifetime;
    float airDrag;
    float bubbleDrag;
    float buoyancy;
    float kineticEnergyWeight;
    float pressureWeight;
    float divergenceWeight;
    float collisionDecay;
    bool useAccurateVelocity;
    char structgen_pad0[3];
};
struct physx_PxParticleSpring {
    uint32_t ind0;
    uint32_t ind1;
    float length;
    float stiffness;
    float damping;
    float pad;
};
struct physx_PxParticleMaterial {
    char structgen_pad0[16];
    void* userData;
};
struct physx_PxSceneDesc;
struct physx_PxPvd;
struct physx_PxOmniPvd;
struct physx_PxPhysics {
    void* vtable_;
};
struct physx_PxActorShape {
    physx_PxRigidActor* actor;
    physx_PxShape* shape;
};
struct physx_PxRaycastHit {
    uint32_t faceIndex;
    uint16_t flags;
    char structgen_pad0[2];
    physx_PxVec3 position;
    physx_PxVec3 normal;
    float distance;
    float u;
    float v;
    char structgen_pad1[4];
    physx_PxRigidActor* actor;
    physx_PxShape* shape;
};
struct physx_PxOverlapHit {
    uint32_t faceIndex;
    char structgen_pad0[4];
    physx_PxRigidActor* actor;
    physx_PxShape* shape;
};
struct physx_PxSweepHit {
    uint32_t faceIndex;
    uint16_t flags;
    char structgen_pad0[2];
    physx_PxVec3 position;
    physx_PxVec3 normal;
    float distance;
    char structgen_pad1[4];
    physx_PxRigidActor* actor;
    physx_PxShape* shape;
};
struct physx_PxRaycastCallback {
    char structgen_pad0[8];
    physx_PxRaycastHit block;
    bool hasBlock;
    char structgen_pad1[7];
    physx_PxRaycastHit* touches;
    uint32_t maxNbTouches;
    uint32_t nbTouches;
};
struct physx_PxOverlapCallback {
    char structgen_pad0[8];
    physx_PxOverlapHit block;
    bool hasBlock;
    char structgen_pad1[7];
    physx_PxOverlapHit* touches;
    uint32_t maxNbTouches;
    uint32_t nbTouches;
};
struct physx_PxSweepCallback {
    char structgen_pad0[8];
    physx_PxSweepHit block;
    bool hasBlock;
    char structgen_pad1[7];
    physx_PxSweepHit* touches;
    uint32_t maxNbTouches;
    uint32_t nbTouches;
};
struct physx_PxRaycastBuffer {
    char structgen_pad0[8];
    physx_PxRaycastHit block;
    bool hasBlock;
    char structgen_pad1[7];
    physx_PxRaycastHit* touches;
    uint32_t maxNbTouches;
    uint32_t nbTouches;
};
struct physx_PxOverlapBuffer {
    char structgen_pad0[8];
    physx_PxOverlapHit block;
    bool hasBlock;
    char structgen_pad1[7];
    physx_PxOverlapHit* touches;
    uint32_t maxNbTouches;
    uint32_t nbTouches;
};
struct physx_PxSweepBuffer {
    char structgen_pad0[8];
    physx_PxSweepHit block;
    bool hasBlock;
    char structgen_pad1[7];
    physx_PxSweepHit* touches;
    uint32_t maxNbTouches;
    uint32_t nbTouches;
};
struct physx_PxQueryCache {
    physx_PxShape* shape;
    physx_PxRigidActor* actor;
    uint32_t faceIndex;
    char structgen_pad0[4];
};
struct physx_PxQueryFilterData {
    physx_PxFilterData data;
    uint16_t flags;
    char structgen_pad0[2];
};
struct physx_PxQueryFilterCallback {
    void* vtable_;
};
struct physx_PxRigidDynamic {
    char structgen_pad0[16];
    void* userData;
};
struct physx_PxRigidStatic {
    char structgen_pad0[16];
    void* userData;
};
struct physx_PxSceneQuerySystem;
struct physx_PxSceneQueryDesc {
    int32_t staticStructure;
    int32_t dynamicStructure;
    uint32_t dynamicTreeRebuildRateHint;
    int32_t dynamicTreeSecondaryPruner;
    int32_t staticBVHBuildStrategy;
    int32_t dynamicBVHBuildStrategy;
    uint32_t staticNbObjectsPerNode;
    uint32_t dynamicNbObjectsPerNode;
    int32_t sceneQueryUpdateMode;
};
struct physx_PxSceneQuerySystemBase {
    void* vtable_;
};
struct physx_PxSceneSQSystem {
    void* vtable_;
};
struct physx_PxSceneQuerySystem {
    void* vtable_;
};
struct physx_PxBroadPhaseRegion {
    physx_PxBounds3 mBounds;
    void* mUserData;
};
struct physx_PxBroadPhaseRegionInfo {
    physx_PxBroadPhaseRegion mRegion;
    uint32_t mNbStaticObjects;
    uint32_t mNbDynamicObjects;
    bool mActive;
    bool mOverlap;
    char structgen_pad0[6];
};
struct physx_PxBroadPhaseCaps {
    uint32_t mMaxNbRegions;
};
struct physx_PxBroadPhaseDesc {
    int32_t mType;
    char structgen_pad0[4];
    uint64_t mContextID;
    char structgen_pad1[8];
    uint32_t mFoundLostPairsCapacity;
    bool mDiscardStaticVsKinematic;
    bool mDiscardKinematicVsKinematic;
    char structgen_pad2[2];
};
struct physx_PxBroadPhaseUpdateData {
    uint32_t const* mCreated;
    uint32_t mNbCreated;
    char structgen_pad0[4];
    uint32_t const* mUpdated;
    uint32_t mNbUpdated;
    char structgen_pad1[4];
    uint32_t const* mRemoved;
    uint32_t mNbRemoved;
    char structgen_pad2[4];
    physx_PxBounds3 const* mBounds;
    uint32_t const* mGroups;
    float const* mDistances;
    uint32_t mCapacity;
    char structgen_pad3[4];
};
struct physx_PxBroadPhasePair {
    uint32_t mID0;
    uint32_t mID1;
};
struct physx_PxBroadPhaseResults {
    uint32_t mNbCreatedPairs;
    char structgen_pad0[4];
    physx_PxBroadPhasePair const* mCreatedPairs;
    uint32_t mNbDeletedPairs;
    char structgen_pad1[4];
    physx_PxBroadPhasePair const* mDeletedPairs;
};
struct physx_PxBroadPhaseRegions {
    void* vtable_;
};
struct physx_PxBroadPhase {
    void* vtable_;
};
struct physx_PxAABBManager {
    void* vtable_;
};
struct physx_PxBroadPhaseCallback;
struct physx_PxSimulationEventCallback;
struct physx_PxSceneLimits {
    uint32_t maxNbActors;
    uint32_t maxNbBodies;
    uint32_t maxNbStaticShapes;
    uint32_t maxNbDynamicShapes;
    uint32_t maxNbAggregates;
    uint32_t maxNbConstraints;
    uint32_t maxNbRegions;
    uint32_t maxNbBroadPhaseOverlaps;
};
struct physx_PxgDynamicsMemoryConfig {
    uint32_t tempBufferCapacity;
    uint32_t maxRigidContactCount;
    uint32_t maxRigidPatchCount;
    uint32_t heapCapacity;
    uint32_t foundLostPairsCapacity;
    uint32_t foundLostAggregatePairsCapacity;
    uint32_t totalAggregatePairsCapacity;
    uint32_t maxSoftBodyContacts;
    uint32_t maxFemClothContacts;
    uint32_t maxParticleContacts;
    uint32_t collisionStackSize;
    uint32_t maxHairContacts;
};
struct physx_PxSceneDesc {
    int32_t staticStructure;
    int32_t dynamicStructure;
    uint32_t dynamicTreeRebuildRateHint;
    int32_t dynamicTreeSecondaryPruner;
    int32_t staticBVHBuildStrategy;
    int32_t dynamicBVHBuildStrategy;
    uint32_t staticNbObjectsPerNode;
    uint32_t dynamicNbObjectsPerNode;
    int32_t sceneQueryUpdateMode;
    physx_PxVec3 gravity;
    physx_PxSimulationEventCallback* simulationEventCallback;
    physx_PxContactModifyCallback* contactModifyCallback;
    physx_PxCCDContactModifyCallback* ccdContactModifyCallback;
    void const* filterShaderData;
    uint32_t filterShaderDataSize;
    char structgen_pad0[4];
    void * filterShader;
    physx_PxSimulationFilterCallback* filterCallback;
    int32_t kineKineFilteringMode;
    int32_t staticKineFilteringMode;
    int32_t broadPhaseType;
    char structgen_pad1[4];
    physx_PxBroadPhaseCallback* broadPhaseCallback;
    physx_PxSceneLimits limits;
    int32_t frictionType;
    int32_t solverType;
    float bounceThresholdVelocity;
    float frictionOffsetThreshold;
    float frictionCorrelationDistance;
    uint32_t flags;
    physx_PxCpuDispatcher* cpuDispatcher;
    char structgen_pad2[8];
    void* userData;
    uint32_t solverBatchSize;
    uint32_t solverArticulationBatchSize;
    uint32_t nbContactDataBlocks;
    uint32_t maxNbContactDataBlocks;
    float maxBiasCoefficient;
    uint32_t contactReportStreamBufferSize;
    uint32_t ccdMaxPasses;
    float ccdThreshold;
    float ccdMaxSeparation;
    float wakeCounterResetValue;
    physx_PxBounds3 sanityBounds;
    physx_PxgDynamicsMemoryConfig gpuDynamicsConfig;
    uint32_t gpuMaxNumPartitions;
    uint32_t gpuMaxNumStaticPartitions;
    uint32_t gpuComputeVersion;
    uint32_t contactPairSlabSize;
    physx_PxSceneQuerySystem* sceneQuerySystem;
    char structgen_pad3[8];
};
struct physx_PxSimulationStatistics {
    uint32_t nbActiveConstraints;
    uint32_t nbActiveDynamicBodies;
    uint32_t nbActiveKinematicBodies;
    uint32_t nbStaticBodies;
    uint32_t nbDynamicBodies;
    uint32_t nbKinematicBodies;
    uint32_t nbShapes[11];
    uint32_t nbAggregates;
    uint32_t nbArticulations;
    uint32_t nbAxisSolverConstraints;
    uint32_t compressedContactSize;
    uint32_t requiredContactConstraintMemory;
    uint32_t peakConstraintMemory;
    uint32_t nbDiscreteContactPairsTotal;
    uint32_t nbDiscreteContactPairsWithCacheHits;
    uint32_t nbDiscreteContactPairsWithContacts;
    uint32_t nbNewPairs;
    uint32_t nbLostPairs;
    uint32_t nbNewTouches;
    uint32_t nbLostTouches;
    uint32_t nbPartitions;
    char structgen_pad0[4];
    uint64_t gpuMemParticles;
    uint64_t gpuMemSoftBodies;
    uint64_t gpuMemFEMCloths;
    uint64_t gpuMemHairSystems;
    uint64_t gpuMemHeap;
    uint64_t gpuMemHeapBroadPhase;
    uint64_t gpuMemHeapNarrowPhase;
    uint64_t gpuMemHeapSolver;
    uint64_t gpuMemHeapArticulation;
    uint64_t gpuMemHeapSimulation;
    uint64_t gpuMemHeapSimulationArticulation;
    uint64_t gpuMemHeapSimulationParticles;
    uint64_t gpuMemHeapSimulationSoftBody;
    uint64_t gpuMemHeapSimulationFEMCloth;
    uint64_t gpuMemHeapSimulationHairSystem;
    uint64_t gpuMemHeapParticles;
    uint64_t gpuMemHeapSoftBodies;
    uint64_t gpuMemHeapFEMCloths;
    uint64_t gpuMemHeapHairSystems;
    uint64_t gpuMemHeapOther;
    uint32_t nbBroadPhaseAdds;
    uint32_t nbBroadPhaseRemoves;
    uint32_t nbDiscreteContactPairs[11][11];
    uint32_t nbCCDPairs[11][11];
    uint32_t nbModifiedContactPairs[11][11];
    uint32_t nbTriggerPairs[11][11];
};
struct physx_PxGpuBodyData {
    physx_PxQuat quat;
    physx_PxVec4 pos;
    physx_PxVec4 linVel;
    physx_PxVec4 angVel;
};
struct physx_PxGpuActorPair {
    uint32_t srcIndex;
    char structgen_pad0[4];
    physx_PxNodeIndex nodeIndex;
};
struct physx_PxIndexDataPair {
    uint32_t index;
    char structgen_pad0[4];
    void* data;
};
struct physx_PxPvdSceneClient {
    void* vtable_;
};
struct physx_PxContactPairHeader;
struct physx_PxDominanceGroupPair {
    uint8_t dominance0;
    uint8_t dominance1;
};
struct physx_PxBroadPhaseCallback {
    void* vtable_;
};
struct physx_PxScene {
    char structgen_pad0[8];
    void* userData;
};
struct physx_PxSceneReadLock {
    char structgen_pad0[8];
};
struct physx_PxSceneWriteLock {
    char structgen_pad0[8];
};
struct physx_PxContactPairExtraDataItem {
    uint8_t type;
};
struct physx_PxContactPairVelocity {
    uint8_t type;
    char structgen_pad0[3];
    physx_PxVec3 linearVelocity[2];
    physx_PxVec3 angularVelocity[2];
};
struct physx_PxContactPairPose {
    uint8_t type;
    char structgen_pad0[3];
    physx_PxTransform globalPose[2];
};
struct physx_PxContactPairIndex {
    uint8_t type;
    char structgen_pad0[1];
    uint16_t index;
};
struct physx_PxContactPairExtraDataIterator {
    uint8_t const* currPtr;
    uint8_t const* endPtr;
    physx_PxContactPairVelocity const* preSolverVelocity;
    physx_PxContactPairVelocity const* postSolverVelocity;
    physx_PxContactPairPose const* eventPose;
    uint32_t contactPairIndex;
    char structgen_pad0[4];
};
struct physx_PxContactPair;
struct physx_PxContactPairHeader {
    physx_PxActor* actors[2];
    uint8_t const* extraDataStream;
    uint16_t extraDataStreamSize;
    uint16_t flags;
    char structgen_pad0[4];
    physx_PxContactPair const* pairs;
    uint32_t nbPairs;
    char structgen_pad1[4];
};
struct physx_PxContactPairPoint {
    physx_PxVec3 position;
    float separation;
    physx_PxVec3 normal;
    uint32_t internalFaceIndex0;
    physx_PxVec3 impulse;
    uint32_t internalFaceIndex1;
};
struct physx_PxContactPair {
    physx_PxShape* shapes[2];
    uint8_t const* contactPatches;
    uint8_t const* contactPoints;
    float const* contactImpulses;
    uint32_t requiredBufferSize;
    uint8_t contactCount;
    uint8_t patchCount;
    uint16_t contactStreamSize;
    uint16_t flags;
    uint16_t events;
    uint32_t internalData[2];
    char structgen_pad0[4];
};
struct physx_PxTriggerPair {
    physx_PxShape* triggerShape;
    physx_PxActor* triggerActor;
    physx_PxShape* otherShape;
    physx_PxActor* otherActor;
    int32_t status;
    uint8_t flags;
    char structgen_pad0[3];
};
struct physx_PxConstraintInfo {
    physx_PxConstraint* constraint;
    void* externalReference;
    uint32_t type;
    char structgen_pad0[4];
};
struct physx_PxSimulationEventCallback {
    void* vtable_;
};
struct physx_PxFEMParameters {
    float velocityDamping;
    float settlingThreshold;
    float sleepThreshold;
    float sleepDamping;
    float selfCollisionFilterDistance;
    float selfCollisionStressTolerance;
};
struct physx_PxPruningStructure {
    char structgen_pad0[16];
};
struct physx_PxExtendedVec3 {
    double x;
    double y;
    double z;
};
struct physx_PxControllerManager;
struct physx_PxObstacle {
    char structgen_pad0[8];
    void* mUserData;
    physx_PxExtendedVec3 mPos;
    physx_PxQuat mRot;
};
struct physx_PxBoxObstacle {
    char structgen_pad0[8];
    void* mUserData;
    physx_PxExtendedVec3 mPos;
    physx_PxQuat mRot;
    physx_PxVec3 mHalfExtents;
    char structgen_pad1[4];
};
struct physx_PxCapsuleObstacle {
    char structgen_pad0[8];
    void* mUserData;
    physx_PxExtendedVec3 mPos;
    physx_PxQuat mRot;
    float mHalfHeight;
    float mRadius;
};
struct physx_PxObstacleContext {
    void* vtable_;
};
struct physx_PxController;
struct physx_PxControllerBehaviorCallback;
struct physx_PxControllerState {
    physx_PxVec3 deltaXP;
    char structgen_pad0[4];
    physx_PxShape* touchedShape;
    physx_PxRigidActor* touchedActor;
    uint32_t touchedObstacleHandle;
    uint32_t collisionFlags;
    bool standOnAnotherCCT;
    bool standOnObstacle;
    bool isMovingUp;
    char structgen_pad1[5];
};
struct physx_PxControllerStats {
    uint16_t nbIterations;
    uint16_t nbFullUpdates;
    uint16_t nbPartialUpdates;
    uint16_t nbTessellation;
};
struct physx_PxControllerHit {
    physx_PxController* controller;
    physx_PxExtendedVec3 worldPos;
    physx_PxVec3 worldNormal;
    physx_PxVec3 dir;
    float length;
    char structgen_pad0[4];
};
struct physx_PxControllerShapeHit {
    physx_PxController* controller;
    physx_PxExtendedVec3 worldPos;
    physx_PxVec3 worldNormal;
    physx_PxVec3 dir;
    float length;
    char structgen_pad0[4];
    physx_PxShape* shape;
    physx_PxRigidActor* actor;
    uint32_t triangleIndex;
    char structgen_pad1[4];
};
struct physx_PxControllersHit {
    physx_PxController* controller;
    physx_PxExtendedVec3 worldPos;
    physx_PxVec3 worldNormal;
    physx_PxVec3 dir;
    float length;
    char structgen_pad0[4];
    physx_PxController* other;
};
struct physx_PxControllerObstacleHit {
    physx_PxController* controller;
    physx_PxExtendedVec3 worldPos;
    physx_PxVec3 worldNormal;
    physx_PxVec3 dir;
    float length;
    char structgen_pad0[4];
    void const* userData;
};
struct physx_PxUserControllerHitReport {
    void* vtable_;
};
struct physx_PxControllerFilterCallback {
    void* vtable_;
};
struct physx_PxControllerFilters {
    physx_PxFilterData const* mFilterData;
    physx_PxQueryFilterCallback* mFilterCallback;
    uint16_t mFilterFlags;
    char structgen_pad0[6];
    physx_PxControllerFilterCallback* mCCTFilterCallback;
};
struct physx_PxControllerDesc {
    char structgen_pad0[8];
    physx_PxExtendedVec3 position;
    physx_PxVec3 upDirection;
    float slopeLimit;
    float invisibleWallHeight;
    float maxJumpHeight;
    float contactOffset;
    float stepOffset;
    float density;
    float scaleCoeff;
    float volumeGrowth;
    char structgen_pad1[4];
    physx_PxUserControllerHitReport* reportCallback;
    physx_PxControllerBehaviorCallback* behaviorCallback;
    int32_t nonWalkableMode;
    char structgen_pad2[4];
    physx_PxMaterial* material;
    bool registerDeletionListener;
    uint8_t clientID;
    char structgen_pad3[6];
    void* userData;
    char structgen_pad4[8];
};
struct physx_PxController {
    void* vtable_;
};
struct physx_PxBoxControllerDesc {
    char structgen_pad0[8];
    physx_PxExtendedVec3 position;
    physx_PxVec3 upDirection;
    float slopeLimit;
    float invisibleWallHeight;
    float maxJumpHeight;
    float contactOffset;
    float stepOffset;
    float density;
    float scaleCoeff;
    float volumeGrowth;
    char structgen_pad1[4];
    physx_PxUserControllerHitReport* reportCallback;
    physx_PxControllerBehaviorCallback* behaviorCallback;
    int32_t nonWalkableMode;
    char structgen_pad2[4];
    physx_PxMaterial* material;
    bool registerDeletionListener;
    uint8_t clientID;
    char structgen_pad3[6];
    void* userData;
    char structgen_pad4[4];
    float halfHeight;
    float halfSideExtent;
    float halfForwardExtent;
};
struct physx_PxBoxController {
    void* vtable_;
};
struct physx_PxCapsuleControllerDesc {
    char structgen_pad0[8];
    physx_PxExtendedVec3 position;
    physx_PxVec3 upDirection;
    float slopeLimit;
    float invisibleWallHeight;
    float maxJumpHeight;
    float contactOffset;
    float stepOffset;
    float density;
    float scaleCoeff;
    float volumeGrowth;
    char structgen_pad1[4];
    physx_PxUserControllerHitReport* reportCallback;
    physx_PxControllerBehaviorCallback* behaviorCallback;
    int32_t nonWalkableMode;
    char structgen_pad2[4];
    physx_PxMaterial* material;
    bool registerDeletionListener;
    uint8_t clientID;
    char structgen_pad3[6];
    void* userData;
    char structgen_pad4[4];
    float radius;
    float height;
    int32_t climbingMode;
};
struct physx_PxCapsuleController {
    void* vtable_;
};
struct physx_PxControllerBehaviorCallback {
    void* vtable_;
};
struct physx_PxControllerManager {
    void* vtable_;
};
struct physx_PxDim3 {
    uint32_t x;
    uint32_t y;
    uint32_t z;
};
struct physx_PxSDFDesc {
    physx_PxBoundedData sdf;
    physx_PxDim3 dims;
    physx_PxVec3 meshLower;
    float spacing;
    uint32_t subgridSize;
    int32_t bitsPerSubgridPixel;
    physx_PxDim3 sdfSubgrids3DTexBlockDim;
    physx_PxBoundedData sdfSubgrids;
    physx_PxBoundedData sdfStartSlots;
    float subgridsMinSdfValue;
    float subgridsMaxSdfValue;
    physx_PxBounds3 sdfBounds;
    float narrowBandThicknessRelativeToSdfBoundsDiagonal;
    uint32_t numThreadsForSdfConstruction;
};
struct physx_PxConvexMeshDesc {
    physx_PxBoundedData points;
    physx_PxBoundedData polygons;
    physx_PxBoundedData indices;
    uint16_t flags;
    uint16_t vertexLimit;
    uint16_t polygonLimit;
    uint16_t quantizedCount;
    physx_PxSDFDesc* sdfDesc;
};
struct physx_PxTriangleMeshDesc {
    physx_PxBoundedData points;
    physx_PxBoundedData triangles;
    uint16_t flags;
    char structgen_pad0[22];
    physx_PxSDFDesc* sdfDesc;
};
struct physx_PxTetrahedronMeshDesc {
    char structgen_pad0[16];
    physx_PxBoundedData points;
    physx_PxBoundedData tetrahedrons;
    uint16_t flags;
    uint16_t tetsPerElement;
    char structgen_pad1[4];
};
struct physx_PxSoftBodySimulationDataDesc {
    physx_PxBoundedData vertexToTet;
};
struct physx_PxBVH34MidphaseDesc {
    uint32_t numPrimsPerLeaf;
    int32_t buildStrategy;
    bool quantized;
    char structgen_pad0[3];
};
struct physx_PxMidphaseDesc {
    char structgen_pad0[16];
};
struct physx_PxBVHDesc {
    physx_PxBoundedData bounds;
    float enlargement;
    uint32_t numPrimsPerLeaf;
    int32_t buildStrategy;
    char structgen_pad0[4];
};
struct physx_PxCookingParams {
    float areaTestEpsilon;
    float planeTolerance;
    int32_t convexMeshCookingType;
    bool suppressTriangleMeshRemapTable;
    bool buildTriangleAdjacencies;
    bool buildGPUData;
    char structgen_pad0[1];
    physx_PxTolerancesScale scale;
    uint32_t meshPreprocessParams;
    float meshWeldTolerance;
    physx_PxMidphaseDesc midphaseDesc;
    uint32_t gaussMapLimit;
    float maxWeightRatioInTet;
};
struct physx_PxDefaultMemoryOutputStream {
    char structgen_pad0[32];
};
struct physx_PxDefaultMemoryInputData {
    char structgen_pad0[32];
};
struct physx_PxDefaultFileOutputStream {
    char structgen_pad0[16];
};
struct physx_PxDefaultFileInputData {
    char structgen_pad0[24];
};
struct physx_PxDefaultAllocator {
    void* vtable_;
};
struct physx_PxJoint;
struct physx_PxRackAndPinionJoint;
struct physx_PxGearJoint;
struct physx_PxD6Joint;
struct physx_PxDistanceJoint;
struct physx_PxContactJoint;
struct physx_PxFixedJoint;
struct physx_PxPrismaticJoint;
struct physx_PxRevoluteJoint;
struct physx_PxSphericalJoint;
struct physx_PxJoint {
    char structgen_pad0[16];
    void* userData;
};
struct physx_PxSpring {
    float stiffness;
    float damping;
};
struct physx_PxDistanceJoint {
    char structgen_pad0[16];
    void* userData;
};
struct physx_PxJacobianRow {
    physx_PxVec3 linear0;
    physx_PxVec3 linear1;
    physx_PxVec3 angular0;
    physx_PxVec3 angular1;
};
struct physx_PxContactJoint {
    char structgen_pad0[16];
    void* userData;
};
struct physx_PxFixedJoint {
    char structgen_pad0[16];
    void* userData;
};
struct physx_PxJointLimitParameters {
    float restitution;
    float bounceThreshold;
    float stiffness;
    float damping;
    float contactDistance_deprecated;
};
struct physx_PxJointLinearLimit {
    float restitution;
    float bounceThreshold;
    float stiffness;
    float damping;
    float contactDistance_deprecated;
    float value;
};
struct physx_PxJointLinearLimitPair {
    float restitution;
    float bounceThreshold;
    float stiffness;
    float damping;
    float contactDistance_deprecated;
    float upper;
    float lower;
};
struct physx_PxJointAngularLimitPair {
    float restitution;
    float bounceThreshold;
    float stiffness;
    float damping;
    float contactDistance_deprecated;
    float upper;
    float lower;
};
struct physx_PxJointLimitCone {
    float restitution;
    float bounceThreshold;
    float stiffness;
    float damping;
    float contactDistance_deprecated;
    float yAngle;
    float zAngle;
};
struct physx_PxJointLimitPyramid {
    float restitution;
    float bounceThreshold;
    float stiffness;
    float damping;
    float contactDistance_deprecated;
    float yAngleMin;
    float yAngleMax;
    float zAngleMin;
    float zAngleMax;
};
struct physx_PxPrismaticJoint {
    char structgen_pad0[16];
    void* userData;
};
struct physx_PxRevoluteJoint {
    char structgen_pad0[16];
    void* userData;
};
struct physx_PxSphericalJoint {
    char structgen_pad0[16];
    void* userData;
};
struct physx_PxD6JointDrive {
    float stiffness;
    float damping;
    float forceLimit;
    uint32_t flags;
};
struct physx_PxD6Joint {
    char structgen_pad0[16];
    void* userData;
};
struct physx_PxGearJoint {
    char structgen_pad0[16];
    void* userData;
};
struct physx_PxRackAndPinionJoint {
    char structgen_pad0[16];
    void* userData;
};
struct physx_PxGroupsMask {
    uint16_t bits0;
    uint16_t bits1;
    uint16_t bits2;
    uint16_t bits3;
};
struct physx_PxDefaultErrorCallback {
    void* vtable_;
};
struct physx_PxRigidActorExt {
    char structgen_pad0[1];
};
struct physx_PxMassProperties {
    physx_PxMat33 inertiaTensor;
    physx_PxVec3 centerOfMass;
    float mass;
};
struct physx_PxRigidBodyExt {
    char structgen_pad0[1];
};
struct physx_PxShapeExt {
    char structgen_pad0[1];
};
struct physx_PxMeshOverlapUtil {
    char structgen_pad0[1040];
};
struct physx_PxBinaryConverter;
struct physx_PxXmlMiscParameter {
    physx_PxVec3 upVector;
    physx_PxTolerancesScale scale;
};
struct physx_PxSerialization {
    char structgen_pad0[1];
};
struct physx_PxDefaultCpuDispatcher {
    void* vtable_;
};
struct physx_PxStringTableExt {
    char structgen_pad0[1];
};
struct physx_PxBroadPhaseExt {
    char structgen_pad0[1];
};
struct physx_PxSceneQueryExt {
    char structgen_pad0[1];
};
struct physx_PxBatchQueryExt {
    void* vtable_;
};
struct physx_PxCustomSceneQuerySystem {
    void* vtable_;
};
struct physx_PxCustomSceneQuerySystemAdapter {
    void* vtable_;
};
struct physx_PxSamplingExt {
    char structgen_pad0[1];
};
struct physx_PxPoissonSampler {
    char structgen_pad0[8];
};
struct physx_PxTriangleMeshPoissonSampler {
    char structgen_pad0[8];
};
struct physx_PxTetrahedronMeshExt {
    char structgen_pad0[1];
};
struct physx_PxRepXObject {
    char const* typeName;
    void const* serializable;
    uint64_t id;
};
struct physx_PxCooking;
struct physx_PxRepXInstantiationArgs {
    char structgen_pad0[8];
    physx_PxCooking* cooker;
    physx_PxStringTable* stringTable;
};
struct physx_XmlMemoryAllocator;
struct physx_XmlWriter;
struct physx_XmlReader;
struct physx_MemoryBuffer;
struct physx_PxRepXSerializer {
    void* vtable_;
};
struct physx_PxVehicleWheels4SimData;
struct physx_PxVehicleWheels4DynData;
struct physx_PxVehicleTireForceCalculator;
struct physx_PxVehicleDrivableSurfaceToTireFrictionPairs;
struct physx_PxVehicleTelemetryData;
struct physx_PxPvdTransport;
struct physx_PxPvd {
    void* vtable_;
};
struct physx_PxPvdTransport {
    void* vtable_;
};
