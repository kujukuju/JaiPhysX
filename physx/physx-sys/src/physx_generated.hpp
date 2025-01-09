/// enum for empty constructor tag
enum PxEMPTY : int32_t {
    PxEmpty = 0,
};

/// enum for zero constructor tag for vectors and matrices
enum PxZERO : int32_t {
    PxZero = 0,
};

/// enum for identity constructor flag for quaternions, transforms, and matrices
enum PxIDENTITY : int32_t {
    PxIdentity = 0,
};

/// Error codes
///
/// These error codes are passed to [`PxErrorCallback`]
enum PxErrorCode : int32_t {
    NoError = 0,
    /// An informational message.
    DebugInfo = 1,
    /// a warning message for the user to help with debugging
    DebugWarning = 2,
    /// method called with invalid parameter(s)
    InvalidParameter = 4,
    /// method was called at a time when an operation is not possible
    InvalidOperation = 8,
    /// method failed to allocate some memory
    OutOfMemory = 16,
    /// The library failed for some reason.
    /// Possibly you have passed invalid values like NaNs, which are not checked for.
    InternalError = 32,
    /// An unrecoverable error, execution should be halted and log output flushed
    Abort = 64,
    /// The SDK has determined that an operation may result in poor performance.
    PerfWarning = 128,
    /// A bit mask for including all errors
    MaskAll = -1,
};

enum PxThreadPriority : uint32_t {
    /// High priority
    High = 0,
    /// Above Normal priority
    AboveNormal = 1,
    /// Normal/default priority
    Normal = 2,
    /// Below Normal priority
    BelowNormal = 3,
    /// Low priority.
    Low = 4,
    ForceDword = 4294967295,
};

/// Default color values used for debug rendering.
enum PxDebugColor : uint32_t {
    ArgbBlack = 4278190080,
    ArgbRed = 4294901760,
    ArgbGreen = 4278255360,
    ArgbBlue = 4278190335,
    ArgbYellow = 4294967040,
    ArgbMagenta = 4294902015,
    ArgbCyan = 4278255615,
    ArgbWhite = 4294967295,
    ArgbGrey = 4286611584,
    ArgbDarkred = 4287102976,
    ArgbDarkgreen = 4278224896,
    ArgbDarkblue = 4278190216,
};

/// an enumeration of concrete classes inheriting from PxBase
///
/// Enumeration space is reserved for future PhysX core types, PhysXExtensions,
/// PhysXVehicle and Custom application types.
enum PxConcreteType : int32_t {
    Undefined = 0,
    Heightfield = 1,
    ConvexMesh = 2,
    TriangleMeshBvh33 = 3,
    TriangleMeshBvh34 = 4,
    TetrahedronMesh = 5,
    SoftbodyMesh = 6,
    RigidDynamic = 7,
    RigidStatic = 8,
    Shape = 9,
    Material = 10,
    SoftbodyMaterial = 11,
    ClothMaterial = 12,
    PbdMaterial = 13,
    FlipMaterial = 14,
    MpmMaterial = 15,
    CustomMaterial = 16,
    Constraint = 17,
    Aggregate = 18,
    ArticulationReducedCoordinate = 19,
    ArticulationLink = 20,
    ArticulationJointReducedCoordinate = 21,
    ArticulationSensor = 22,
    ArticulationSpatialTendon = 23,
    ArticulationFixedTendon = 24,
    ArticulationAttachment = 25,
    ArticulationTendonJoint = 26,
    PruningStructure = 27,
    Bvh = 28,
    SoftBody = 29,
    SoftBodyState = 30,
    PbdParticlesystem = 31,
    FlipParticlesystem = 32,
    MpmParticlesystem = 33,
    CustomParticlesystem = 34,
    FemCloth = 35,
    HairSystem = 36,
    ParticleBuffer = 37,
    ParticleDiffuseBuffer = 38,
    ParticleClothBuffer = 39,
    ParticleRigidBuffer = 40,
    PhysxCoreCount = 41,
    FirstPhysxExtension = 256,
    FirstVehicleExtension = 512,
    FirstUserExtension = 1024,
};

/// Flags for PxBase.
enum PxBaseFlag : int32_t {
    OwnsMemory = 1,
    IsReleasable = 2,
};

enum PxBaseFlags : uint16_t {
    OwnsMemory_Bit = 1 << 0,
    IsReleasable_Bit = 1 << 1,
};

/// Flags used to configure binary meta data entries, typically set through PX_DEF_BIN_METADATA defines.
enum PxMetaDataFlag : int32_t {
    /// declares a class
    Class = 1,
    /// declares class to be virtual
    Virtual = 2,
    /// declares a typedef
    Typedef = 4,
    /// declares a pointer
    Ptr = 8,
    /// declares a handle
    Handle = 16,
    /// declares extra data exported with PxSerializer::exportExtraData
    ExtraData = 32,
    /// specifies one element of extra data
    ExtraItem = 64,
    /// specifies an array of extra data
    ExtraItems = 128,
    /// specifies a name of extra data
    ExtraName = 256,
    /// declares a union
    Union = 512,
    /// declares explicit padding data
    Padding = 1024,
    /// declares aligned data
    Alignment = 2048,
    /// specifies that the count value's most significant bit needs to be masked out
    CountMaskMsb = 4096,
    /// specifies that the count value is treated as zero for a variable value of one - special case for single triangle meshes
    CountSkipIfOne = 8192,
    /// specifies that the control value is the negate of the variable value
    ControlFlip = 16384,
    /// specifies that the control value is masked - mask bits are assumed to be within eCONTROL_MASK_RANGE
    ControlMask = 32768,
    /// mask range allowed for eCONTROL_MASK
    ControlMaskRange = 255,
    ForceDword = 2147483647,
};

/// Identifies the type of each heavyweight PxTask object
enum PxTaskType : int32_t {
    /// PxTask will be run on the CPU
    Cpu = 0,
    /// Return code when attempting to find a task that does not exist
    NotPresent = 1,
    /// PxTask execution has been completed
    Completed = 2,
};

/// A geometry type.
///
/// Used to distinguish the type of a ::PxGeometry object.
enum PxGeometryType : int32_t {
    Sphere = 0,
    Plane = 1,
    Capsule = 2,
    Box = 3,
    Convexmesh = 4,
    Particlesystem = 5,
    Tetrahedronmesh = 6,
    Trianglemesh = 7,
    Heightfield = 8,
    Hairsystem = 9,
    Custom = 10,
    /// internal use only!
    GeometryCount = 11,
    /// internal use only!
    Invalid = -1,
};

/// Geometry-level query flags.
enum PxGeometryQueryFlag : int32_t {
    /// Saves/restores SIMD control word for each query (safer but slower). Omit this if you took care of it yourself in your app.
    SimdGuard = 1,
};

enum PxGeometryQueryFlags : uint32_t {
    SimdGuard_Bit = 1 << 0,
};

/// Desired build strategy for bounding-volume hierarchies
enum PxBVHBuildStrategy : int32_t {
    /// Fast build strategy. Fast build speed, good runtime performance in most cases. Recommended for runtime cooking.
    Fast = 0,
    /// Default build strategy. Medium build speed, good runtime performance in all cases.
    Default = 1,
    /// SAH build strategy. Slower builds, slightly improved runtime performance in some cases.
    Sah = 2,
    Last = 3,
};

/// Flags controlling the simulated behavior of the convex mesh geometry.
///
/// Used in ::PxConvexMeshGeometryFlags.
enum PxConvexMeshGeometryFlag : int32_t {
    /// Use tighter (but more expensive to compute) bounds around the convex geometry.
    TightBounds = 1,
};

enum PxConvexMeshGeometryFlags : uint8_t {
    TightBounds_Bit = 1 << 0,
};

/// Flags controlling the simulated behavior of the triangle mesh geometry.
///
/// Used in ::PxMeshGeometryFlags.
enum PxMeshGeometryFlag : int32_t {
    /// Use tighter (but more expensive to compute) bounds around the triangle mesh geometry.
    TightBounds = 1,
    /// Meshes with this flag set are treated as double-sided.
    /// This flag is currently only used for raycasts and sweeps (it is ignored for overlap queries).
    /// For detailed specifications of this flag for meshes and heightfields please refer to the Geometry Query section of the user guide.
    DoubleSided = 2,
};

enum PxMeshGeometryFlags : uint8_t {
    TightBounds_Bit = 1 << 0,
    DoubleSided_Bit = 1 << 1,
};

/// Identifies the solver to use for a particle system.
enum PxParticleSolverType : int32_t {
    /// The position based dynamics solver that can handle fluid, granular material, cloth, inflatables etc. See [`PxPBDParticleSystem`].
    Pbd = 1,
    /// The FLIP fluid solver. See [`PxFLIPParticleSystem`].
    Flip = 2,
    /// The MPM (material point method) solver that can handle a variety of materials. See [`PxMPMParticleSystem`].
    Mpm = 4,
    /// Custom solver. The user needs to specify the interaction of the particle by providing appropriate functions. Can be used e.g. for molecular dynamics simulations. See [`PxCustomParticleSystem`].
    Custom = 8,
};

/// Scene query and geometry query behavior flags.
///
/// PxHitFlags are used for 3 different purposes:
///
/// 1) To request hit fields to be filled in by scene queries (such as hit position, normal, face index or UVs).
/// 2) Once query is completed, to indicate which fields are valid (note that a query may produce more valid fields than requested).
/// 3) To specify additional options for the narrow phase and mid-phase intersection routines.
///
/// All these flags apply to both scene queries and geometry queries (PxGeometryQuery).
enum PxHitFlag : int32_t {
    /// "position" member of [`PxQueryHit`] is valid
    Position = 1,
    /// "normal" member of [`PxQueryHit`] is valid
    Normal = 2,
    /// "u" and "v" barycentric coordinates of [`PxQueryHit`] are valid. Not applicable to sweep queries.
    Uv = 8,
    /// Performance hint flag for sweeps when it is known upfront there's no initial overlap.
    /// NOTE: using this flag may cause undefined results if shapes are initially overlapping.
    AssumeNoInitialOverlap = 16,
    /// Report any first hit. Used for geometries that contain more than one primitive. For meshes,
    /// if neither eMESH_MULTIPLE nor eANY_HIT is specified, a single closest hit will be reported.
    AnyHit = 32,
    /// Report all hits for meshes rather than just the first. Not applicable to sweep queries.
    MeshMultiple = 64,
    /// Report hits with back faces of mesh triangles. Also report hits for raycast
    /// originating on mesh surface and facing away from the surface normal. Not applicable to sweep queries.
    /// Please refer to the user guide for heightfield-specific differences.
    MeshBothSides = 128,
    /// Use more accurate but slower narrow phase sweep tests.
    /// May provide better compatibility with PhysX 3.2 sweep behavior.
    PreciseSweep = 256,
    /// Report the minimum translation depth, normal and contact point.
    Mtd = 512,
    /// "face index" member of [`PxQueryHit`] is valid
    FaceIndex = 1024,
    Default = 1027,
    /// Only this subset of flags can be modified by pre-filter. Other modifications will be discarded.
    ModifiableFlags = 464,
};

enum PxHitFlags : uint16_t {
    Position_Bit = 1 << 0,
    Normal_Bit = 1 << 1,
    Uv_Bit = 1 << 3,
    AssumeNoInitialOverlap_Bit = 1 << 4,
    AnyHit_Bit = 1 << 5,
    MeshMultiple_Bit = 1 << 6,
    MeshBothSides_Bit = 1 << 7,
    PreciseSweep_Bit = 1 << 8,
    Mtd_Bit = 1 << 9,
    FaceIndex_Bit = 1 << 10,
    Default_Bit = Position | Normal | FaceIndex,
    ModifiableFlags_Bit = AssumeNoInitialOverlap | MeshMultiple | MeshBothSides | PreciseSweep,
};

/// Describes the format of height field samples.
enum PxHeightFieldFormat : int32_t {
    /// Height field height data is 16 bit signed integers, followed by triangle materials.
    ///
    /// Each sample is 32 bits wide arranged as follows:
    ///
    /// 1) First there is a 16 bit height value.
    /// 2) Next, two one byte material indices, with the high bit of each byte reserved for special use.
    /// (so the material index is only 7 bits).
    /// The high bit of material0 is the tess-flag.
    /// The high bit of material1 is reserved for future use.
    ///
    /// There are zero or more unused bytes before the next sample depending on PxHeightFieldDesc.sampleStride,
    /// where the application may eventually keep its own data.
    ///
    /// This is the only format supported at the moment.
    S16Tm = 1,
};

/// Determines the tessellation of height field cells.
enum PxHeightFieldTessFlag : int32_t {
    /// This flag determines which way each quad cell is subdivided.
    ///
    /// The flag lowered indicates subdivision like this: (the 0th vertex is referenced by only one triangle)
    ///
    /// +--+--+--+---> column
    /// | /| /| /|
    /// |/ |/ |/ |
    /// +--+--+--+
    /// | /| /| /|
    /// |/ |/ |/ |
    /// +--+--+--+
    /// |
    /// |
    /// V row
    ///
    /// The flag raised indicates subdivision like this: (the 0th vertex is shared by two triangles)
    ///
    /// +--+--+--+---> column
    /// |
    /// \
    /// |
    /// \
    /// |
    /// \
    /// |
    /// |
    /// \
    /// |
    /// \
    /// |
    /// \
    /// |
    /// +--+--+--+
    /// |
    /// \
    /// |
    /// \
    /// |
    /// \
    /// |
    /// |
    /// \
    /// |
    /// \
    /// |
    /// \
    /// |
    /// +--+--+--+
    /// |
    /// |
    /// V row
    E0ThVertexShared = 1,
};

/// Enum with flag values to be used in PxHeightFieldDesc.flags.
enum PxHeightFieldFlag : int32_t {
    /// Disable collisions with height field with boundary edges.
    ///
    /// Raise this flag if several terrain patches are going to be placed adjacent to each other,
    /// to avoid a bump when sliding across.
    ///
    /// This flag is ignored in contact generation with sphere and capsule shapes.
    NoBoundaryEdges = 1,
};

enum PxHeightFieldFlags : uint16_t {
    NoBoundaryEdges_Bit = 1 << 0,
};

/// Special material index values for height field samples.
enum PxHeightFieldMaterial : int32_t {
    /// A material indicating that the triangle should be treated as a hole in the mesh.
    Hole = 127,
};

enum PxMeshMeshQueryFlag : int32_t {
    /// Report all overlaps
    Default = 0,
    /// Ignore coplanar triangle-triangle overlaps
    DiscardCoplanar = 1,
};

enum PxMeshMeshQueryFlags : uint32_t {
    DiscardCoplanar_Bit = 1 << 0,
};

/// Enum with flag values to be used in PxSimpleTriangleMesh::flags.
enum PxMeshFlag : int32_t {
    /// Specifies if the SDK should flip normals.
    ///
    /// The PhysX libraries assume that the face normal of a triangle with vertices [a,b,c] can be computed as:
    /// edge1 = b-a
    /// edge2 = c-a
    /// face_normal = edge1 x edge2.
    ///
    /// Note: This is the same as a counterclockwise winding in a right handed coordinate system or
    /// alternatively a clockwise winding order in a left handed coordinate system.
    ///
    /// If this does not match the winding order for your triangles, raise the below flag.
    Flipnormals = 1,
    /// Denotes the use of 16-bit vertex indices
    E16BitIndices = 2,
};

enum PxMeshFlags : uint16_t {
    Flipnormals_Bit = 1 << 0,
    E16BitIndices_Bit = 1 << 1,
};

/// Mesh midphase structure. This enum is used to select the desired acceleration structure for midphase queries
/// (i.e. raycasts, overlaps, sweeps vs triangle meshes).
///
/// The PxMeshMidPhase::eBVH33 structure is the one used in recent PhysX versions (up to PhysX 3.3). It has great performance and is
/// supported on all platforms. It is deprecated since PhysX 5.x.
///
/// The PxMeshMidPhase::eBVH34 structure is a revisited implementation introduced in PhysX 3.4. It can be significantly faster both
/// in terms of cooking performance and runtime performance.
enum PxMeshMidPhase : int32_t {
    /// Default midphase mesh structure, as used up to PhysX 3.3 (deprecated)
    Bvh33 = 0,
    /// New midphase mesh structure, introduced in PhysX 3.4
    Bvh34 = 1,
    Last = 2,
};

/// Flags for the mesh geometry properties.
///
/// Used in ::PxTriangleMeshFlags.
enum PxTriangleMeshFlag : int32_t {
    /// The triangle mesh has 16bits vertex indices.
    E16BitIndices = 2,
    /// The triangle mesh has adjacency information build.
    AdjacencyInfo = 4,
    /// Indicates that this mesh would preferably not be the mesh projected for mesh-mesh collision. This can indicate that the mesh is not well tessellated.
    PreferNoSdfProj = 8,
};

enum PxTriangleMeshFlags : uint8_t {
    E16BitIndices_Bit = 1 << 1,
    AdjacencyInfo_Bit = 1 << 2,
    PreferNoSdfProj_Bit = 1 << 3,
};

enum PxTetrahedronMeshFlag : int32_t {
    /// The tetrahedron mesh has 16bits vertex indices
    E16BitIndices = 2,
};

enum PxTetrahedronMeshFlags : uint8_t {
    E16BitIndices_Bit = 1 << 1,
};

/// Flags which control the behavior of an actor.
enum PxActorFlag : int32_t {
    /// Enable debug renderer for this actor
    Visualization = 1,
    /// Disables scene gravity for this actor
    DisableGravity = 2,
    /// Enables the sending of PxSimulationEventCallback::onWake() and PxSimulationEventCallback::onSleep() notify events
    SendSleepNotifies = 4,
    /// Disables simulation for the actor.
    ///
    /// This is only supported by PxRigidStatic and PxRigidDynamic actors and can be used to reduce the memory footprint when rigid actors are
    /// used for scene queries only.
    ///
    /// Setting this flag will remove all constraints attached to the actor from the scene.
    ///
    /// If this flag is set, the following calls are forbidden:
    ///
    /// PxRigidBody: setLinearVelocity(), setAngularVelocity(), addForce(), addTorque(), clearForce(), clearTorque(), setForceAndTorque()
    ///
    /// PxRigidDynamic: setKinematicTarget(), setWakeCounter(), wakeUp(), putToSleep()
    ///
    /// Sleeping:
    /// Raising this flag will set all velocities and the wake counter to 0, clear all forces, clear the kinematic target, put the actor
    /// to sleep and wake up all touching actors from the previous frame.
    DisableSimulation = 8,
};

enum PxActorFlags : uint8_t {
    Visualization_Bit = 1 << 0,
    DisableGravity_Bit = 1 << 1,
    SendSleepNotifies_Bit = 1 << 2,
    DisableSimulation_Bit = 1 << 3,
};

/// Identifies each type of actor.
enum PxActorType : int32_t {
    /// A static rigid body
    RigidStatic = 0,
    /// A dynamic rigid body
    RigidDynamic = 1,
    /// An articulation link
    ArticulationLink = 2,
};

enum PxAggregateType : int32_t {
    /// Aggregate will contain various actors of unspecified types
    Generic = 0,
    /// Aggregate will only contain static actors
    Static = 1,
    /// Aggregate will only contain kinematic actors
    Kinematic = 2,
};

/// Constraint row flags
///
/// These flags configure the post-processing of constraint rows and the behavior of the solver while solving constraints
enum Px1DConstraintFlag : int32_t {
    /// whether the constraint is a spring. Mutually exclusive with eRESTITUTION. If set, eKEEPBIAS is ignored.
    Spring = 1,
    /// whether the constraint is a force or acceleration spring. Only valid if eSPRING is set.
    AccelerationSpring = 2,
    /// whether the restitution model should be applied to generate the target velocity. Mutually exclusive with eSPRING. If restitution causes a bounces, eKEEPBIAS is ignored
    Restitution = 4,
    /// whether to keep the error term when solving for velocity. Ignored if restitution generates bounce, or eSPRING is set.
    Keepbias = 8,
    /// whether to accumulate the force value from this constraint in the force total that is reported for the constraint and tested for breakage
    OutputForce = 16,
    /// whether the constraint has a drive force limit (which will be scaled by dt unless PxConstraintFlag::eLIMITS_ARE_FORCES is set)
    HasDriveLimit = 32,
    /// whether this is an angular or linear constraint
    AngularConstraint = 64,
    /// whether the constraint's geometric error should drive the target velocity
    DriveRow = 128,
};

enum Px1DConstraintFlags : uint16_t {
    Spring_Bit = 1 << 0,
    AccelerationSpring_Bit = 1 << 1,
    Restitution_Bit = 1 << 2,
    Keepbias_Bit = 1 << 3,
    OutputForce_Bit = 1 << 4,
    HasDriveLimit_Bit = 1 << 5,
    AngularConstraint_Bit = 1 << 6,
    DriveRow_Bit = 1 << 7,
};

/// Constraint type hints which the solver uses to optimize constraint handling
enum PxConstraintSolveHint : int32_t {
    /// no special properties
    None = 0,
    /// a group of acceleration drive constraints with the same stiffness and drive parameters
    Acceleration1 = 256,
    /// temporary special value to identify SLERP drive rows
    SlerpSpring = 258,
    /// a group of acceleration drive constraints with the same stiffness and drive parameters
    Acceleration2 = 512,
    /// a group of acceleration drive constraints with the same stiffness and drive parameters
    Acceleration3 = 768,
    /// rotational equality constraints with no force limit and no velocity target
    RotationalEquality = 1024,
    /// rotational inequality constraints with (0, PX_MAX_FLT) force limits
    RotationalInequality = 1025,
    /// equality constraints with no force limit and no velocity target
    Equality = 2048,
    /// inequality constraints with (0, PX_MAX_FLT) force limits
    Inequality = 2049,
};

/// Flags for determining which components of the constraint should be visualized.
enum PxConstraintVisualizationFlag : int32_t {
    /// visualize constraint frames
    LocalFrames = 1,
    /// visualize constraint limits
    Limits = 2,
};

/// Flags for determining how PVD should serialize a constraint update
enum PxPvdUpdateType : int32_t {
    /// triggers createPvdInstance call, creates an instance of a constraint
    CreateInstance = 0,
    /// triggers releasePvdInstance call, releases an instance of a constraint
    ReleaseInstance = 1,
    /// triggers updatePvdProperties call, updates all properties of a constraint
    UpdateAllProperties = 2,
    /// triggers simUpdate call, updates all simulation properties of a constraint
    UpdateSimProperties = 3,
};

/// Constraint descriptor used inside the solver
enum ConstraintType : int32_t {
    /// Defines this pair is a contact constraint
    ContactConstraint = 0,
    /// Defines this pair is a joint constraint
    JointConstraint = 1,
};

/// Data structure used for preparing constraints before solving them
enum BodyState : int32_t {
    DynamicBody = 1,
    StaticBody = 2,
    KinematicBody = 4,
    Articulation = 8,
};

/// @
/// {
enum PxArticulationAxis : int32_t {
    /// Rotational about eX
    Twist = 0,
    /// Rotational about eY
    Swing1 = 1,
    /// Rotational about eZ
    Swing2 = 2,
    /// Linear in eX
    X = 3,
    /// Linear in eY
    Y = 4,
    /// Linear in eZ
    Z = 5,
    Count = 6,
};

enum PxArticulationMotion : int32_t {
    /// Locked axis, i.e. degree of freedom (DOF)
    Locked = 0,
    /// Limited DOF - set limits of joint DOF together with this flag, see PxArticulationJointReducedCoordinate::setLimitParams
    Limited = 1,
    /// Free DOF
    Free = 2,
};

enum PxArticulationMotions : uint8_t {
    Limited_Bit = 1 << 0,
    Free_Bit = 1 << 1,
};

enum PxArticulationJointType : int32_t {
    /// All joint axes, i.e. degrees of freedom (DOFs) locked
    Fix = 0,
    /// Single linear DOF, e.g. cart on a rail
    Prismatic = 1,
    /// Single rotational DOF, e.g. an elbow joint or a rotational motor, position wrapped at 2pi radians
    Revolute = 2,
    /// Single rotational DOF, e.g. an elbow joint or a rotational motor, position not wrapped
    RevoluteUnwrapped = 3,
    /// Ball and socket joint with two or three DOFs
    Spherical = 4,
    Undefined = 5,
};

enum PxArticulationFlag : int32_t {
    /// Set articulation base to be fixed.
    FixBase = 1,
    /// Limits for drive effort are forces and torques rather than impulses, see PxArticulationDrive::maxForce.
    DriveLimitsAreForces = 2,
    /// Disable collisions between the articulation's links (note that parent/child collisions are disabled internally in either case).
    DisableSelfCollision = 4,
    /// Enable in order to be able to query joint solver (i.e. constraint) forces using PxArticulationCache::jointSolverForces.
    ComputeJointForces = 8,
};

enum PxArticulationFlags : uint8_t {
    FixBase_Bit = 1 << 0,
    DriveLimitsAreForces_Bit = 1 << 1,
    DisableSelfCollision_Bit = 1 << 2,
    ComputeJointForces_Bit = 1 << 3,
};

enum PxArticulationDriveType : int32_t {
    /// The output of the implicit spring drive controller is a force/torque.
    Force = 0,
    /// The output of the implicit spring drive controller is a joint acceleration (use this to get (spatial)-inertia-invariant behavior of the drive).
    Acceleration = 1,
    /// Sets the drive gains internally to track a target position almost kinematically (i.e. with very high drive gains).
    Target = 2,
    /// Sets the drive gains internally to track a target velocity almost kinematically (i.e. with very high drive gains).
    Velocity = 3,
    None = 4,
};

/// A description of the types of articulation data that may be directly written to and read from the GPU using the functions
/// PxScene::copyArticulationData() and PxScene::applyArticulationData(). Types that are read-only may only be used in conjunction with
/// PxScene::copyArticulationData(). Types that are write-only may only be used in conjunction with PxScene::applyArticulationData().
/// A subset of data types may be used in conjunction with both PxScene::applyArticulationData() and PxScene::applyArticulationData().
enum PxArticulationGpuDataType : int32_t {
    /// The joint positions, read and write, see PxScene::copyArticulationData(), PxScene::applyArticulationData()
    JointPosition = 0,
    /// The joint velocities, read and write,  see PxScene::copyArticulationData(), PxScene::applyArticulationData()
    JointVelocity = 1,
    /// The joint accelerations, read only, see PxScene::copyArticulationData()
    JointAcceleration = 2,
    /// The applied joint forces, write only, see PxScene::applyArticulationData()
    JointForce = 3,
    /// The computed joint constraint solver forces, read only, see PxScene::copyArticulationData()()
    JointSolverForce = 4,
    /// The velocity targets for the joint drives, write only, see PxScene::applyArticulationData()
    JointTargetVelocity = 5,
    /// The position targets for the joint drives, write only, see PxScene::applyArticulationData()
    JointTargetPosition = 6,
    /// The spatial sensor forces, read only, see PxScene::copyArticulationData()
    SensorForce = 7,
    /// The root link transform, read and write, see PxScene::copyArticulationData(), PxScene::applyArticulationData()
    RootTransform = 8,
    /// The root link velocity, read and write, see PxScene::copyArticulationData(), PxScene::applyArticulationData()
    RootVelocity = 9,
    /// The link transforms including root link, read only, see PxScene::copyArticulationData()
    LinkTransform = 10,
    /// The link velocities including root link, read only, see PxScene::copyArticulationData()
    LinkVelocity = 11,
    /// The forces to apply to links, write only, see PxScene::applyArticulationData()
    LinkForce = 12,
    /// The torques to apply to links, write only, see PxScene::applyArticulationData()
    LinkTorque = 13,
    /// Fixed tendon data, write only, see PxScene::applyArticulationData()
    FixedTendon = 14,
    /// Fixed tendon joint data, write only, see PxScene::applyArticulationData()
    FixedTendonJoint = 15,
    /// Spatial tendon data, write only, see PxScene::applyArticulationData()
    SpatialTendon = 16,
    /// Spatial tendon attachment data, write only, see PxScene::applyArticulationData()
    SpatialTendonAttachment = 17,
};

/// These flags determine what data is read or written to the internal articulation data via cache.
enum PxArticulationCacheFlag : int32_t {
    /// The joint velocities, see PxArticulationCache::jointVelocity.
    Velocity = 1,
    /// The joint accelerations, see PxArticulationCache::jointAcceleration.
    Acceleration = 2,
    /// The joint positions, see PxArticulationCache::jointPosition.
    Position = 4,
    /// The joint forces, see PxArticulationCache::jointForce.
    Force = 8,
    /// The link velocities, see PxArticulationCache::linkVelocity.
    LinkVelocity = 16,
    /// The link accelerations, see PxArticulationCache::linkAcceleration.
    LinkAcceleration = 32,
    /// Root link transform, see PxArticulationCache::rootLinkData.
    RootTransform = 64,
    /// Root link velocities (read/write) and accelerations (read), see PxArticulationCache::rootLinkData.
    RootVelocities = 128,
    /// The spatial sensor forces, see PxArticulationCache::sensorForces.
    SensorForces = 256,
    /// Solver constraint joint forces, see PxArticulationCache::jointSolverForces.
    JointSolverForces = 512,
    All = 247,
};

enum PxArticulationCacheFlags : uint32_t {
    Velocity_Bit = 1 << 0,
    Acceleration_Bit = 1 << 1,
    Position_Bit = 1 << 2,
    Force_Bit = 1 << 3,
    LinkVelocity_Bit = 1 << 4,
    LinkAcceleration_Bit = 1 << 5,
    RootTransform_Bit = 1 << 6,
    RootVelocities_Bit = 1 << 7,
    SensorForces_Bit = 1 << 8,
    JointSolverForces_Bit = 1 << 9,
    All_Bit = Velocity | Acceleration | Position | LinkVelocity | LinkAcceleration | RootTransform | RootVelocities,
};

/// Flags to configure the forces reported by articulation link sensors.
enum PxArticulationSensorFlag : int32_t {
    /// Raise to receive forces from forward dynamics.
    ForwardDynamicsForces = 1,
    /// Raise to receive forces from constraint solver.
    ConstraintSolverForces = 2,
    /// Raise to receive forces in the world rotation frame, otherwise they will be reported in the sensor's local frame.
    WorldFrame = 4,
};

enum PxArticulationSensorFlags : uint8_t {
    ForwardDynamicsForces_Bit = 1 << 0,
    ConstraintSolverForces_Bit = 1 << 1,
    WorldFrame_Bit = 1 << 2,
};

/// Flag that configures articulation-state updates by PxArticulationReducedCoordinate::updateKinematic.
enum PxArticulationKinematicFlag : int32_t {
    /// Raise after any changes to the articulation root or joint positions using non-cache API calls. Updates links' positions and velocities.
    Position = 1,
    /// Raise after velocity-only changes to the articulation root or joints using non-cache API calls. Updates links' velocities.
    Velocity = 2,
};

enum PxArticulationKinematicFlags : uint8_t {
    Position_Bit = 1 << 0,
    Velocity_Bit = 1 << 1,
};

/// Flags which affect the behavior of PxShapes.
enum PxShapeFlag : int32_t {
    /// The shape will partake in collision in the physical simulation.
    ///
    /// It is illegal to raise the eSIMULATION_SHAPE and eTRIGGER_SHAPE flags.
    /// In the event that one of these flags is already raised the sdk will reject any
    /// attempt to raise the other.  To raise the eSIMULATION_SHAPE first ensure that
    /// eTRIGGER_SHAPE is already lowered.
    ///
    /// This flag has no effect if simulation is disabled for the corresponding actor (see [`PxActorFlag::eDISABLE_SIMULATION`]).
    SimulationShape = 1,
    /// The shape will partake in scene queries (ray casts, overlap tests, sweeps, ...).
    SceneQueryShape = 2,
    /// The shape is a trigger which can send reports whenever other shapes enter/leave its volume.
    ///
    /// Triangle meshes and heightfields can not be triggers. Shape creation will fail in these cases.
    ///
    /// Shapes marked as triggers do not collide with other objects. If an object should act both
    /// as a trigger shape and a collision shape then create a rigid body with two shapes, one being a
    /// trigger shape and the other a collision shape. It is illegal to raise the eTRIGGER_SHAPE and
    /// eSIMULATION_SHAPE flags on a single PxShape instance.  In the event that one of these flags is already
    /// raised the sdk will reject any attempt to raise the other.  To raise the eTRIGGER_SHAPE flag first
    /// ensure that eSIMULATION_SHAPE flag is already lowered.
    ///
    /// Trigger shapes will no longer send notification events for interactions with other trigger shapes.
    ///
    /// Shapes marked as triggers are allowed to participate in scene queries, provided the eSCENE_QUERY_SHAPE flag is set.
    ///
    /// This flag has no effect if simulation is disabled for the corresponding actor (see [`PxActorFlag::eDISABLE_SIMULATION`]).
    TriggerShape = 4,
    /// Enable debug renderer for this shape
    Visualization = 8,
};

enum PxShapeFlags : uint8_t {
    SimulationShape_Bit = 1 << 0,
    SceneQueryShape_Bit = 1 << 1,
    TriggerShape_Bit = 1 << 2,
    Visualization_Bit = 1 << 3,
};

/// Parameter to addForce() and addTorque() calls, determines the exact operation that is carried out.
enum PxForceMode : int32_t {
    /// parameter has unit of mass * length / time^2, i.e., a force
    Force = 0,
    /// parameter has unit of mass * length / time, i.e., force * time
    Impulse = 1,
    /// parameter has unit of length / time, i.e., the effect is mass independent: a velocity change.
    VelocityChange = 2,
    /// parameter has unit of length/ time^2, i.e., an acceleration. It gets treated just like a force except the mass is not divided out before integration.
    Acceleration = 3,
};

/// Collection of flags describing the behavior of a rigid body.
enum PxRigidBodyFlag : int32_t {
    /// Enable kinematic mode for the body.
    Kinematic = 1,
    /// Use the kinematic target transform for scene queries.
    ///
    /// If this flag is raised, then scene queries will treat the kinematic target transform as the current pose
    /// of the body (instead of using the actual pose). Without this flag, the kinematic target will only take
    /// effect with respect to scene queries after a simulation step.
    UseKinematicTargetForSceneQueries = 2,
    /// Enable CCD for the body.
    EnableCcd = 4,
    /// Enabled CCD in swept integration for the actor.
    ///
    /// If this flag is raised and CCD is enabled, CCD interactions will simulate friction. By default, friction is disabled in CCD interactions because
    /// CCD friction has been observed to introduce some simulation artifacts. CCD friction was enabled in previous versions of the SDK. Raising this flag will result in behavior
    /// that is a closer match for previous versions of the SDK.
    ///
    /// This flag requires PxRigidBodyFlag::eENABLE_CCD to be raised to have any effect.
    EnableCcdFriction = 8,
    /// Register a rigid body to dynamically adjust contact offset based on velocity. This can be used to achieve a CCD effect.
    ///
    /// If both eENABLE_CCD and eENABLE_SPECULATIVE_CCD are set on the same body, then angular motions are handled by speculative
    /// contacts (eENABLE_SPECULATIVE_CCD) while linear motions are handled by sweeps (eENABLE_CCD).
    EnableSpeculativeCcd = 16,
    /// Register a rigid body for reporting pose changes by the simulation at an early stage.
    ///
    /// Sometimes it might be advantageous to get access to the new pose of a rigid body as early as possible and
    /// not wait until the call to fetchResults() returns. Setting this flag will schedule the rigid body to get reported
    /// in [`PxSimulationEventCallback::onAdvance`](). Please refer to the documentation of that callback to understand
    /// the behavior and limitations of this functionality.
    EnablePoseIntegrationPreview = 32,
    /// Permit CCD to limit maxContactImpulse. This is useful for use-cases like a destruction system but can cause visual artefacts so is not enabled by default.
    EnableCcdMaxContactImpulse = 64,
    /// Carries over forces/accelerations between frames, rather than clearing them
    RetainAccelerations = 128,
    /// Forces kinematic-kinematic pairs notifications for this actor.
    ///
    /// This flag overrides the global scene-level PxPairFilteringMode setting for kinematic actors.
    /// This is equivalent to having PxPairFilteringMode::eKEEP for pairs involving this actor.
    ///
    /// A particular use case is when you have a large amount of kinematic actors, but you are only
    /// interested in interactions between a few of them. In this case it is best to use
    /// PxSceneDesc.kineKineFilteringMode = PxPairFilteringMode::eKILL, and then raise the
    /// eFORCE_KINE_KINE_NOTIFICATIONS flag on the small set of kinematic actors that need
    /// notifications.
    ///
    /// This has no effect if PxRigidBodyFlag::eKINEMATIC is not set.
    ///
    /// Changing this flag at runtime will not have an effect until you remove and re-add the actor to the scene.
    ForceKineKineNotifications = 256,
    /// Forces static-kinematic pairs notifications for this actor.
    ///
    /// Similar to eFORCE_KINE_KINE_NOTIFICATIONS, but for static-kinematic interactions.
    ///
    /// This has no effect if PxRigidBodyFlag::eKINEMATIC is not set.
    ///
    /// Changing this flag at runtime will not have an effect until you remove and re-add the actor to the scene.
    ForceStaticKineNotifications = 512,
    /// Enables computation of gyroscopic forces on the rigid body.
    EnableGyroscopicForces = 1024,
};

enum PxRigidBodyFlags : uint16_t {
    Kinematic_Bit = 1 << 0,
    UseKinematicTargetForSceneQueries_Bit = 1 << 1,
    EnableCcd_Bit = 1 << 2,
    EnableCcdFriction_Bit = 1 << 3,
    EnableSpeculativeCcd_Bit = 1 << 4,
    EnablePoseIntegrationPreview_Bit = 1 << 5,
    EnableCcdMaxContactImpulse_Bit = 1 << 6,
    RetainAccelerations_Bit = 1 << 7,
    ForceKineKineNotifications_Bit = 1 << 8,
    ForceStaticKineNotifications_Bit = 1 << 9,
    EnableGyroscopicForces_Bit = 1 << 10,
};

/// constraint flags
///
/// eBROKEN is a read only flag
enum PxConstraintFlag : int32_t {
    /// whether the constraint is broken
    Broken = 1,
    /// whether actor1 should get projected to actor0 for this constraint (note: projection of a static/kinematic actor to a dynamic actor will be ignored)
    ProjectToActor0 = 2,
    /// whether actor0 should get projected to actor1 for this constraint (note: projection of a static/kinematic actor to a dynamic actor will be ignored)
    ProjectToActor1 = 4,
    /// whether the actors should get projected for this constraint (the direction will be chosen by PhysX)
    Projection = 6,
    /// whether contacts should be generated between the objects this constraint constrains
    CollisionEnabled = 8,
    /// whether this constraint should be visualized, if constraint visualization is turned on
    Visualization = 16,
    /// limits for drive strength are forces rather than impulses
    DriveLimitsAreForces = 32,
    /// perform preprocessing for improved accuracy on D6 Slerp Drive (this flag will be removed in a future release when preprocessing is no longer required)
    ImprovedSlerp = 128,
    /// suppress constraint preprocessing, intended for use with rowResponseThreshold. May result in worse solver accuracy for ill-conditioned constraints.
    DisablePreprocessing = 256,
    /// enables extended limit ranges for angular limits (e.g., limit values > PxPi or
    /// <
    /// -PxPi)
    EnableExtendedLimits = 512,
    /// the constraint type is supported by gpu dynamics
    GpuCompatible = 1024,
    /// updates the constraint each frame
    AlwaysUpdate = 2048,
    /// disables the constraint. SolverPrep functions won't be called for this constraint.
    DisableConstraint = 4096,
};

enum PxConstraintFlags : uint16_t {
    Broken_Bit = 1 << 0,
    ProjectToActor0_Bit = 1 << 1,
    ProjectToActor1_Bit = 1 << 2,
    Projection_Bit = ProjectToActor0 | ProjectToActor1,
    CollisionEnabled_Bit = 1 << 3,
    Visualization_Bit = 1 << 4,
    DriveLimitsAreForces_Bit = 1 << 5,
    ImprovedSlerp_Bit = 1 << 7,
    DisablePreprocessing_Bit = 1 << 8,
    EnableExtendedLimits_Bit = 1 << 9,
    GpuCompatible_Bit = 1 << 10,
    AlwaysUpdate_Bit = 1 << 11,
    DisableConstraint_Bit = 1 << 12,
};

/// Header for a contact patch where all points share same material and normal
enum PxContactPatchFlags : int32_t {
    /// Indicates this contact stream has face indices.
    HasFaceIndices = 1,
    /// Indicates this contact stream is modifiable.
    Modifiable = 2,
    /// Indicates this contact stream is notify-only (no contact response).
    ForceNoResponse = 4,
    /// Indicates this contact stream has modified mass ratios
    HasModifiedMassRatios = 8,
    /// Indicates this contact stream has target velocities set
    HasTargetVelocity = 16,
    /// Indicates this contact stream has max impulses set
    HasMaxImpulse = 32,
    /// Indicates this contact stream needs patches re-generated. This is required if the application modified either the contact normal or the material properties
    RegeneratePatches = 64,
    CompressedModifiedContact = 128,
};

/// A class to iterate over a compressed contact stream. This supports read-only access to the various contact formats.
enum StreamFormat : int32_t {
    SimpleStream = 0,
    ModifiableStream = 1,
    CompressedModifiableStream = 2,
};

/// Flags specifying deletion event types.
enum PxDeletionEventFlag : int32_t {
    /// The user has called release on an object.
    UserRelease = 1,
    /// The destructor of an object has been called and the memory has been released.
    MemoryRelease = 2,
};

enum PxDeletionEventFlags : uint8_t {
    UserRelease_Bit = 1 << 0,
    MemoryRelease_Bit = 1 << 1,
};

/// Collection of flags describing the actions to take for a collision pair.
enum PxPairFlag : int32_t {
    /// Process the contacts of this collision pair in the dynamics solver.
    ///
    /// Only takes effect if the colliding actors are rigid bodies.
    SolveContact = 1,
    /// Call contact modification callback for this collision pair
    ///
    /// Only takes effect if the colliding actors are rigid bodies.
    ModifyContacts = 2,
    /// Call contact report callback or trigger callback when this collision pair starts to be in contact.
    ///
    /// If one of the two collision objects is a trigger shape (see [`PxShapeFlag::eTRIGGER_SHAPE`])
    /// then the trigger callback will get called as soon as the other object enters the trigger volume.
    /// If none of the two collision objects is a trigger shape then the contact report callback will get
    /// called when the actors of this collision pair start to be in contact.
    ///
    /// Only takes effect if the colliding actors are rigid bodies.
    ///
    /// Only takes effect if eDETECT_DISCRETE_CONTACT or eDETECT_CCD_CONTACT is raised
    NotifyTouchFound = 4,
    /// Call contact report callback while this collision pair is in contact
    ///
    /// If none of the two collision objects is a trigger shape then the contact report callback will get
    /// called while the actors of this collision pair are in contact.
    ///
    /// Triggers do not support this event. Persistent trigger contacts need to be tracked separately by observing eNOTIFY_TOUCH_FOUND/eNOTIFY_TOUCH_LOST events.
    ///
    /// Only takes effect if the colliding actors are rigid bodies.
    ///
    /// No report will get sent if the objects in contact are sleeping.
    ///
    /// Only takes effect if eDETECT_DISCRETE_CONTACT or eDETECT_CCD_CONTACT is raised
    ///
    /// If this flag gets enabled while a pair is in touch already, there will be no eNOTIFY_TOUCH_PERSISTS events until the pair loses and regains touch.
    NotifyTouchPersists = 8,
    /// Call contact report callback or trigger callback when this collision pair stops to be in contact
    ///
    /// If one of the two collision objects is a trigger shape (see [`PxShapeFlag::eTRIGGER_SHAPE`])
    /// then the trigger callback will get called as soon as the other object leaves the trigger volume.
    /// If none of the two collision objects is a trigger shape then the contact report callback will get
    /// called when the actors of this collision pair stop to be in contact.
    ///
    /// Only takes effect if the colliding actors are rigid bodies.
    ///
    /// This event will also get triggered if one of the colliding objects gets deleted.
    ///
    /// Only takes effect if eDETECT_DISCRETE_CONTACT or eDETECT_CCD_CONTACT is raised
    NotifyTouchLost = 16,
    /// Call contact report callback when this collision pair is in contact during CCD passes.
    ///
    /// If CCD with multiple passes is enabled, then a fast moving object might bounce on and off the same
    /// object multiple times. Hence, the same pair might be in contact multiple times during a simulation step.
    /// This flag will make sure that all the detected collision during CCD will get reported. For performance
    /// reasons, the system can not always tell whether the contact pair lost touch in one of the previous CCD
    /// passes and thus can also not always tell whether the contact is new or has persisted. eNOTIFY_TOUCH_CCD
    /// just reports when the two collision objects were detected as being in contact during a CCD pass.
    ///
    /// Only takes effect if the colliding actors are rigid bodies.
    ///
    /// Trigger shapes are not supported.
    ///
    /// Only takes effect if eDETECT_CCD_CONTACT is raised
    NotifyTouchCcd = 32,
    /// Call contact report callback when the contact force between the actors of this collision pair exceeds one of the actor-defined force thresholds.
    ///
    /// Only takes effect if the colliding actors are rigid bodies.
    ///
    /// Only takes effect if eDETECT_DISCRETE_CONTACT or eDETECT_CCD_CONTACT is raised
    NotifyThresholdForceFound = 64,
    /// Call contact report callback when the contact force between the actors of this collision pair continues to exceed one of the actor-defined force thresholds.
    ///
    /// Only takes effect if the colliding actors are rigid bodies.
    ///
    /// If a pair gets re-filtered and this flag has previously been disabled, then the report will not get fired in the same frame even if the force threshold has been reached in the
    /// previous one (unless [`eNOTIFY_THRESHOLD_FORCE_FOUND`] has been set in the previous frame).
    ///
    /// Only takes effect if eDETECT_DISCRETE_CONTACT or eDETECT_CCD_CONTACT is raised
    NotifyThresholdForcePersists = 128,
    /// Call contact report callback when the contact force between the actors of this collision pair falls below one of the actor-defined force thresholds (includes the case where this collision pair stops being in contact).
    ///
    /// Only takes effect if the colliding actors are rigid bodies.
    ///
    /// If a pair gets re-filtered and this flag has previously been disabled, then the report will not get fired in the same frame even if the force threshold has been reached in the
    /// previous one (unless [`eNOTIFY_THRESHOLD_FORCE_FOUND`] or #eNOTIFY_THRESHOLD_FORCE_PERSISTS has been set in the previous frame).
    ///
    /// Only takes effect if eDETECT_DISCRETE_CONTACT or eDETECT_CCD_CONTACT is raised
    NotifyThresholdForceLost = 256,
    /// Provide contact points in contact reports for this collision pair.
    ///
    /// Only takes effect if the colliding actors are rigid bodies and if used in combination with the flags eNOTIFY_TOUCH_... or eNOTIFY_THRESHOLD_FORCE_...
    ///
    /// Only takes effect if eDETECT_DISCRETE_CONTACT or eDETECT_CCD_CONTACT is raised
    NotifyContactPoints = 512,
    /// This flag is used to indicate whether this pair generates discrete collision detection contacts.
    ///
    /// Contacts are only responded to if eSOLVE_CONTACT is enabled.
    DetectDiscreteContact = 1024,
    /// This flag is used to indicate whether this pair generates CCD contacts.
    ///
    /// The contacts will only be responded to if eSOLVE_CONTACT is enabled on this pair.
    ///
    /// The scene must have PxSceneFlag::eENABLE_CCD enabled to use this feature.
    ///
    /// Non-static bodies of the pair should have PxRigidBodyFlag::eENABLE_CCD specified for this feature to work correctly.
    ///
    /// This flag is not supported with trigger shapes. However, CCD trigger events can be emulated using non-trigger shapes
    /// and requesting eNOTIFY_TOUCH_FOUND and eNOTIFY_TOUCH_LOST and not raising eSOLVE_CONTACT on the pair.
    DetectCcdContact = 2048,
    /// Provide pre solver velocities in contact reports for this collision pair.
    ///
    /// If the collision pair has contact reports enabled, the velocities of the rigid bodies before contacts have been solved
    /// will be provided in the contact report callback unless the pair lost touch in which case no data will be provided.
    ///
    /// Usually it is not necessary to request these velocities as they will be available by querying the velocity from the provided
    /// PxRigidActor object directly. However, it might be the case that the velocity of a rigid body gets set while the simulation is running
    /// in which case the PxRigidActor would return this new velocity in the contact report callback and not the velocity the simulation used.
    PreSolverVelocity = 4096,
    /// Provide post solver velocities in contact reports for this collision pair.
    ///
    /// If the collision pair has contact reports enabled, the velocities of the rigid bodies after contacts have been solved
    /// will be provided in the contact report callback unless the pair lost touch in which case no data will be provided.
    PostSolverVelocity = 8192,
    /// Provide rigid body poses in contact reports for this collision pair.
    ///
    /// If the collision pair has contact reports enabled, the rigid body poses at the contact event will be provided
    /// in the contact report callback unless the pair lost touch in which case no data will be provided.
    ///
    /// Usually it is not necessary to request these poses as they will be available by querying the pose from the provided
    /// PxRigidActor object directly. However, it might be the case that the pose of a rigid body gets set while the simulation is running
    /// in which case the PxRigidActor would return this new pose in the contact report callback and not the pose the simulation used.
    /// Another use case is related to CCD with multiple passes enabled, A fast moving object might bounce on and off the same
    /// object multiple times. This flag can be used to request the rigid body poses at the time of impact for each such collision event.
    ContactEventPose = 16384,
    /// For internal use only.
    NextFree = 32768,
    /// Provided default flag to do simple contact processing for this collision pair.
    ContactDefault = 1025,
    /// Provided default flag to get commonly used trigger behavior for this collision pair.
    TriggerDefault = 1044,
};

enum PxPairFlags : uint16_t {
    SolveContact_Bit = 1 << 0,
    ModifyContacts_Bit = 1 << 1,
    NotifyTouchFound_Bit = 1 << 2,
    NotifyTouchPersists_Bit = 1 << 3,
    NotifyTouchLost_Bit = 1 << 4,
    NotifyTouchCcd_Bit = 1 << 5,
    NotifyThresholdForceFound_Bit = 1 << 6,
    NotifyThresholdForcePersists_Bit = 1 << 7,
    NotifyThresholdForceLost_Bit = 1 << 8,
    NotifyContactPoints_Bit = 1 << 9,
    DetectDiscreteContact_Bit = 1 << 10,
    DetectCcdContact_Bit = 1 << 11,
    PreSolverVelocity_Bit = 1 << 12,
    PostSolverVelocity_Bit = 1 << 13,
    ContactEventPose_Bit = 1 << 14,
    NextFree_Bit = 1 << 15,
    ContactDefault_Bit = SolveContact | DetectDiscreteContact,
    TriggerDefault_Bit = NotifyTouchFound | NotifyTouchLost | DetectDiscreteContact,
};

/// Collection of flags describing the filter actions to take for a collision pair.
enum PxFilterFlag : int32_t {
    /// Ignore the collision pair as long as the bounding volumes of the pair objects overlap.
    ///
    /// Killed pairs will be ignored by the simulation and won't run through the filter again until one
    /// of the following occurs:
    ///
    /// The bounding volumes of the two objects overlap again (after being separated)
    ///
    /// The user enforces a re-filtering (see [`PxScene::resetFiltering`]())
    Kill = 1,
    /// Ignore the collision pair as long as the bounding volumes of the pair objects overlap or until filtering relevant data changes for one of the collision objects.
    ///
    /// Suppressed pairs will be ignored by the simulation and won't make another filter request until one
    /// of the following occurs:
    ///
    /// Same conditions as for killed pairs (see [`eKILL`])
    ///
    /// The filter data or the filter object attributes change for one of the collision objects
    Suppress = 2,
    /// Invoke the filter callback ([`PxSimulationFilterCallback::pairFound`]()) for this collision pair.
    Callback = 4,
    /// Track this collision pair with the filter callback mechanism.
    ///
    /// When the bounding volumes of the collision pair lose contact, the filter callback [`PxSimulationFilterCallback::pairLost`]()
    /// will be invoked. Furthermore, the filter status of the collision pair can be adjusted through [`PxSimulationFilterCallback::statusChange`]()
    /// once per frame (until a pairLost() notification occurs).
    Notify = 12,
    /// Provided default to get standard behavior:
    ///
    /// The application configure the pair's collision properties once when bounding volume overlap is found and
    /// doesn't get asked again about that pair until overlap status or filter properties changes, or re-filtering is requested.
    ///
    /// No notification is provided when bounding volume overlap is lost
    ///
    /// The pair will not be killed or suppressed, so collision detection will be processed
    Default = 0,
};

enum PxFilterFlags : uint16_t {
    Kill_Bit = 1 << 0,
    Suppress_Bit = 1 << 1,
    Callback_Bit = 1 << 2,
    Notify_Bit = Callback,
};

/// Identifies each type of filter object.
enum PxFilterObjectType : int32_t {
    /// A static rigid body
    RigidStatic = 0,
    /// A dynamic rigid body
    RigidDynamic = 1,
    /// An articulation
    Articulation = 2,
    /// A particle system
    Particlesystem = 3,
    /// A FEM-based soft body
    Softbody = 4,
    /// A FEM-based cloth
    ///
    /// In development
    Femcloth = 5,
    /// A hair system
    ///
    /// In development
    Hairsystem = 6,
    /// internal use only!
    MaxTypeCount = 16,
    /// internal use only!
    Undefined = 15,
};

enum PxFilterObjectFlag : int32_t {
    Kinematic = 16,
    Trigger = 32,
};

enum PxPairFilteringMode : int32_t {
    /// Output pair from BP, potentially send to user callbacks, create regular interaction object.
    ///
    /// Enable contact pair filtering between kinematic/static or kinematic/kinematic rigid bodies.
    ///
    /// By default contacts between these are suppressed (see [`PxFilterFlag::eSUPPRESS`]) and don't get reported to the filter mechanism.
    /// Use this mode if these pairs should go through the filtering pipeline nonetheless.
    ///
    /// This mode is not mutable, and must be set in PxSceneDesc at scene creation.
    Keep = 0,
    /// Output pair from BP, create interaction marker. Can be later switched to regular interaction.
    Suppress = 1,
    /// Don't output pair from BP. Cannot be later switched to regular interaction, needs "resetFiltering" call.
    Kill = 2,
};

enum PxDataAccessFlag : int32_t {
    Readable = 1,
    Writable = 2,
    Device = 4,
};

enum PxDataAccessFlags : uint8_t {
    Readable_Bit = 1 << 0,
    Writable_Bit = 1 << 1,
    Device_Bit = 1 << 2,
};

/// Flags which control the behavior of a material.
enum PxMaterialFlag : int32_t {
    /// If this flag is set, friction computations are always skipped between shapes with this material and any other shape.
    DisableFriction = 1,
    /// Whether to use strong friction.
    /// The difference between "normal" and "strong" friction is that the strong friction feature
    /// remembers the "friction error" between simulation steps. The friction is a force trying to
    /// hold objects in place (or slow them down) and this is handled in the solver. But since the
    /// solver is only an approximation, the result of the friction calculation can include a small
    /// "error" - e.g. a box resting on a slope should not move at all if the static friction is in
    /// action, but could slowly glide down the slope because of a small friction error in each
    /// simulation step. The strong friction counter-acts this by remembering the small error and
    /// taking it to account during the next simulation step.
    ///
    /// However, in some cases the strong friction could cause problems, and this is why it is
    /// possible to disable the strong friction feature by setting this flag. One example is
    /// raycast vehicles that are sliding fast across the surface, but still need a precise
    /// steering behavior. It may be a good idea to reenable the strong friction when objects
    /// are coming to a rest, to prevent them from slowly creeping down inclines.
    ///
    /// Note: This flag only has an effect if the PxMaterialFlag::eDISABLE_FRICTION bit is 0.
    DisableStrongFriction = 2,
    /// Whether to use the patch friction model.
    /// This flag only has an effect if PxFrictionType::ePATCH friction model is used.
    ///
    /// When using the patch friction model, up to 2 friction anchors are generated per patch. As the number of friction anchors
    /// can be smaller than the number of contacts, the normal force is accumulated over all contacts and used to compute friction
    /// for all anchors. Where there are more than 2 anchors, this can produce frictional behavior that is too strong (approximately 2x as strong
    /// as analytical models suggest).
    ///
    /// This flag causes the normal force to be distributed between the friction anchors such that the total amount of friction applied does not
    /// exceed the analytical results.
    ImprovedPatchFriction = 4,
    /// This flag has the effect of enabling an implicit spring model for the normal force computation.
    CompliantContact = 8,
};

enum PxMaterialFlags : uint16_t {
    DisableFriction_Bit = 1 << 0,
    DisableStrongFriction_Bit = 1 << 1,
    ImprovedPatchFriction_Bit = 1 << 2,
    CompliantContact_Bit = 1 << 3,
};

/// Enumeration that determines the way in which two material properties will be combined to yield a friction or restitution coefficient for a collision.
///
/// When two actors come in contact with each other, they each have materials with various coefficients, but we only need a single set of coefficients for the pair.
///
/// Physics doesn't have any inherent combinations because the coefficients are determined empirically on a case by case
/// basis. However, simulating this with a pairwise lookup table is often impractical.
///
/// For this reason the following combine behaviors are available:
///
/// eAVERAGE
/// eMIN
/// eMULTIPLY
/// eMAX
///
/// The effective combine mode for the pair is maximum(material0.combineMode, material1.combineMode).
enum PxCombineMode : int32_t {
    /// Average: (a + b)/2
    Average = 0,
    /// Minimum: minimum(a,b)
    Min = 1,
    /// Multiply: a*b
    Multiply = 2,
    /// Maximum: maximum(a,b)
    Max = 3,
    /// This is not a valid combine mode, it is a sentinel to denote the number of possible values. We assert that the variable's value is smaller than this.
    NValues = 4,
    /// This is not a valid combine mode, it is to assure that the size of the enum type is big enough.
    Pad32 = 2147483647,
};

/// Identifies dirty particle buffers that need to be updated in the particle system.
///
/// This flag can be used mark the device user buffers that are dirty and need to be written to the particle system.
enum PxParticleBufferFlag : int32_t {
    /// No data specified
    None = 0,
    /// Specifies the position (first 3 floats) and inverse mass (last float) data (array of PxVec4 * number of particles)
    UpdatePosition = 1,
    /// Specifies the velocity (first 3 floats) data (array of PxVec4 * number of particles)
    UpdateVelocity = 2,
    /// Specifies the per-particle phase flag data (array of PxU32 * number of particles)
    UpdatePhase = 4,
    /// Specifies the rest position (first 3 floats) data for cloth buffers
    UpdateRestposition = 8,
    /// Specifies the cloth buffer (see PxParticleClothBuffer)
    UpdateCloth = 32,
    /// Specifies the rigid buffer (see PxParticleRigidBuffer)
    UpdateRigid = 64,
    /// Specifies the diffuse particle parameter buffer (see PxDiffuseParticleParams)
    UpdateDiffuseParam = 128,
    /// Specifies the attachments.
    UpdateAttachments = 256,
    All = 495,
};

enum PxParticleBufferFlags : uint32_t {
    UpdatePosition_Bit = 1 << 0,
    UpdateVelocity_Bit = 1 << 1,
    UpdatePhase_Bit = 1 << 2,
    UpdateRestposition_Bit = 1 << 3,
    UpdateCloth_Bit = 1 << 5,
    UpdateRigid_Bit = 1 << 6,
    UpdateDiffuseParam_Bit = 1 << 7,
    UpdateAttachments_Bit = 1 << 8,
    All_Bit = UpdatePosition | UpdateVelocity | UpdatePhase | UpdateRestposition | UpdateCloth | UpdateRigid | UpdateDiffuseParam | UpdateAttachments,
};

/// Identifies per-particle behavior for a PxParticleSystem.
///
/// See [`PxParticleSystem::createPhase`]().
enum PxParticlePhaseFlag : uint32_t {
    /// Bits [ 0, 19] represent the particle group for controlling collisions
    ParticlePhaseGroupMask = 1048575,
    /// Bits [20, 23] hold flags about how the particle behave
    ParticlePhaseFlagsMask = 4293918720,
    /// If set this particle will interact with particles of the same group
    ParticlePhaseSelfCollide = 1048576,
    /// If set this particle will ignore collisions with particles closer than the radius in the rest pose, this flag should not be specified unless valid rest positions have been specified using setRestParticles()
    ParticlePhaseSelfCollideFilter = 2097152,
    /// If set this particle will generate fluid density constraints for its overlapping neighbors
    ParticlePhaseFluid = 4194304,
};

enum PxParticlePhaseFlags : uint32_t {
    ParticlePhaseGroupMask_Bit = 0x000fffff,
    ParticlePhaseFlagsMask_Bit = ParticlePhaseSelfCollide | ParticlePhaseSelfCollideFilter | ParticlePhaseFluid,
    ParticlePhaseSelfCollide_Bit = 1 << 20,
    ParticlePhaseSelfCollideFilter_Bit = 1 << 21,
    ParticlePhaseFluid_Bit = 1 << 22,
};

/// Specifies memory space for a PxBuffer instance.
enum PxBufferType : int32_t {
    Host = 0,
    Device = 1,
};

/// Filtering flags for scene queries.
enum PxQueryFlag : int32_t {
    /// Traverse static shapes
    Static = 1,
    /// Traverse dynamic shapes
    Dynamic = 2,
    /// Run the pre-intersection-test filter (see [`PxQueryFilterCallback::preFilter`]())
    Prefilter = 4,
    /// Run the post-intersection-test filter (see [`PxQueryFilterCallback::postFilter`]())
    Postfilter = 8,
    /// Abort traversal as soon as any hit is found and return it via callback.block.
    /// Helps query performance. Both eTOUCH and eBLOCK hitTypes are considered hits with this flag.
    AnyHit = 16,
    /// All hits are reported as touching. Overrides eBLOCK returned from user filters with eTOUCH.
    /// This is also an optimization hint that may improve query performance.
    NoBlock = 32,
    /// Same as eBATCH_QUERY_LEGACY_BEHAVIOUR, more explicit name making it clearer that this can also be used
    /// with regular/non-batched queries if needed.
    DisableHardcodedFilter = 64,
    /// Reserved for internal use
    Reserved = 32768,
};

enum PxQueryFlags : uint16_t {
    Static_Bit = 1 << 0,
    Dynamic_Bit = 1 << 1,
    Prefilter_Bit = 1 << 2,
    Postfilter_Bit = 1 << 3,
    AnyHit_Bit = 1 << 4,
    NoBlock_Bit = 1 << 5,
    DisableHardcodedFilter_Bit = 1 << 6,
    Reserved_Bit = 1 << 15,
};

/// Classification of scene query hits (intersections).
///
/// - eNONE: Returning this hit type means that the hit should not be reported.
/// - eBLOCK: For all raycast, sweep and overlap queries the nearest eBLOCK type hit will always be returned in PxHitCallback::block member.
/// - eTOUCH: Whenever a raycast, sweep or overlap query was called with non-zero PxHitCallback::nbTouches and PxHitCallback::touches
/// parameters, eTOUCH type hits that are closer or same distance (touchDistance
/// <
/// = blockDistance condition)
/// as the globally nearest eBLOCK type hit, will be reported.
/// - For example, to record all hits from a raycast query, always return eTOUCH.
///
/// All hits in overlap() queries are treated as if the intersection distance were zero.
/// This means the hits are unsorted and all eTOUCH hits are recorded by the callback even if an eBLOCK overlap hit was encountered.
/// Even though all overlap() blocking hits have zero length, only one (arbitrary) eBLOCK overlap hit is recorded in PxHitCallback::block.
/// All overlap() eTOUCH type hits are reported (zero touchDistance
/// <
/// = zero blockDistance condition).
///
/// For raycast/sweep/overlap calls with zero touch buffer or PxHitCallback::nbTouches member,
/// only the closest hit of type eBLOCK is returned. All eTOUCH hits are discarded.
enum PxQueryHitType : int32_t {
    /// the query should ignore this shape
    None = 0,
    /// a hit on the shape touches the intersection geometry of the query but does not block it
    Touch = 1,
    /// a hit on the shape blocks the query (does not block overlap queries)
    Block = 2,
};

/// Collection of flags providing a mechanism to lock motion along/around a specific axis.
enum PxRigidDynamicLockFlag : int32_t {
    LockLinearX = 1,
    LockLinearY = 2,
    LockLinearZ = 4,
    LockAngularX = 8,
    LockAngularY = 16,
    LockAngularZ = 32,
};

enum PxRigidDynamicLockFlags : uint8_t {
    LockLinearX_Bit = 1 << 0,
    LockLinearY_Bit = 1 << 1,
    LockLinearZ_Bit = 1 << 2,
    LockAngularX_Bit = 1 << 3,
    LockAngularY_Bit = 1 << 4,
    LockAngularZ_Bit = 1 << 5,
};

/// Pruning structure used to accelerate scene queries.
///
/// eNONE uses a simple data structure that consumes less memory than the alternatives,
/// but generally has slower query performance.
///
/// eDYNAMIC_AABB_TREE usually provides the fastest queries. However there is a
/// constant per-frame management cost associated with this structure. How much work should
/// be done per frame can be tuned via the [`PxSceneQueryDesc::dynamicTreeRebuildRateHint`]
/// parameter.
///
/// eSTATIC_AABB_TREE is typically used for static objects. It is the same as the
/// dynamic AABB tree, without the per-frame overhead. This can be a good choice for static
/// objects, if no static objects are added, moved or removed after the scene has been
/// created. If there is no such guarantee (e.g. when streaming parts of the world in and out),
/// then the dynamic version is a better choice even for static objects.
enum PxPruningStructureType : int32_t {
    /// Using a simple data structure
    None = 0,
    /// Using a dynamic AABB tree
    DynamicAabbTree = 1,
    /// Using a static AABB tree
    StaticAabbTree = 2,
    Last = 3,
};

/// Secondary pruning structure used for newly added objects in dynamic trees.
///
/// Dynamic trees (PxPruningStructureType::eDYNAMIC_AABB_TREE) are slowly rebuilt
/// over several frames. A secondary pruning structure holds and manages objects
/// added to the scene while this rebuild is in progress.
///
/// eNONE ignores newly added objects. This means that for a number of frames (roughly
/// defined by PxSceneQueryDesc::dynamicTreeRebuildRateHint) newly added objects will
/// be ignored by scene queries. This can be acceptable when streaming large worlds, e.g.
/// when the objects added at the boundaries of the game world don't immediately need to be
/// visible from scene queries (it would be equivalent to streaming that data in a few frames
/// later). The advantage of this approach is that there is no CPU cost associated with
/// inserting the new objects in the scene query data structures, and no extra runtime cost
/// when performing queries.
///
/// eBUCKET uses a structure similar to PxPruningStructureType::eNONE. Insertion is fast but
/// query cost can be high.
///
/// eINCREMENTAL uses an incremental AABB-tree, with no direct PxPruningStructureType equivalent.
/// Query time is fast but insertion cost can be high.
///
/// eBVH uses a PxBVH structure. This usually offers the best overall performance.
enum PxDynamicTreeSecondaryPruner : int32_t {
    /// no secondary pruner, new objects aren't visible to SQ for a few frames
    None = 0,
    /// bucket-based secondary pruner, faster updates, slower query time
    Bucket = 1,
    /// incremental-BVH secondary pruner, faster query time, slower updates
    Incremental = 2,
    /// PxBVH-based secondary pruner, good overall performance
    Bvh = 3,
    Last = 4,
};

/// Scene query update mode
///
/// This enum controls what work is done when the scene query system is updated. The updates traditionally happen when PxScene::fetchResults
/// is called. This function then calls PxSceneQuerySystem::finalizeUpdates, where the update mode is used.
///
/// fetchResults/finalizeUpdates will sync changed bounds during simulation and update the scene query bounds in pruners, this work is mandatory.
///
/// eBUILD_ENABLED_COMMIT_ENABLED does allow to execute the new AABB tree build step during fetchResults/finalizeUpdates, additionally
/// the pruner commit is called where any changes are applied. During commit PhysX refits the dynamic scene query tree and if a new tree
/// was built and the build finished the tree is swapped with current AABB tree.
///
/// eBUILD_ENABLED_COMMIT_DISABLED does allow to execute the new AABB tree build step during fetchResults/finalizeUpdates. Pruner commit
/// is not called, this means that refit will then occur during the first scene query following fetchResults/finalizeUpdates, or may be forced
/// by the method PxScene::flushQueryUpdates() / PxSceneQuerySystemBase::flushUpdates().
///
/// eBUILD_DISABLED_COMMIT_DISABLED no further scene query work is executed. The scene queries update needs to be called manually, see
/// PxScene::sceneQueriesUpdate (see that function's doc for the equivalent PxSceneQuerySystem sequence). It is recommended to call
/// PxScene::sceneQueriesUpdate right after fetchResults/finalizeUpdates as the pruning structures are not updated.
enum PxSceneQueryUpdateMode : int32_t {
    /// Both scene query build and commit are executed.
    BuildEnabledCommitEnabled = 0,
    /// Scene query build only is executed.
    BuildEnabledCommitDisabled = 1,
    /// No work is done, no update of scene queries
    BuildDisabledCommitDisabled = 2,
};

/// Built-in enum for default PxScene pruners
///
/// This is passed as a pruner index to various functions in the following APIs.
enum PxScenePrunerIndex : uint32_t {
    PxScenePrunerStatic = 0,
    PxScenePrunerDynamic = 1,
    PxSceneCompoundPruner = 4294967295,
};

/// Broad phase algorithm used in the simulation
///
/// eSAP is a good generic choice with great performance when many objects are sleeping. Performance
/// can degrade significantly though, when all objects are moving, or when large numbers of objects
/// are added to or removed from the broad phase. This algorithm does not need world bounds to be
/// defined in order to work.
///
/// eMBP is an alternative broad phase algorithm that does not suffer from the same performance
/// issues as eSAP when all objects are moving or when inserting large numbers of objects. However
/// its generic performance when many objects are sleeping might be inferior to eSAP, and it requires
/// users to define world bounds in order to work.
///
/// eABP is a revisited implementation of MBP, which automatically manages broad-phase regions.
/// It offers the convenience of eSAP (no need to define world bounds or regions) and the performance
/// of eMBP when a lot of objects are moving. While eSAP can remain faster when most objects are
/// sleeping and eMBP can remain faster when it uses a large number of properly-defined regions,
/// eABP often gives the best performance on average and the best memory usage.
///
/// ePABP is a parallel implementation of ABP. It can often be the fastest (CPU) broadphase, but it
/// can use more memory than ABP.
///
/// eGPU is a GPU implementation of the incremental sweep and prune approach. Additionally, it uses a ABP-style
/// initial pair generation approach to avoid large spikes when inserting shapes. It not only has the advantage
/// of traditional SAP approch which is good for when many objects are sleeping, but due to being fully parallel,
/// it also is great when lots of shapes are moving or for runtime pair insertion and removal. It can become a
/// performance bottleneck if there are a very large number of shapes roughly projecting to the same values
/// on a given axis. If the scene has a very large number of shapes in an actor, e.g. a humanoid, it is recommended
/// to use an aggregate to represent multi-shape or multi-body actors to minimize stress placed on the broad phase.
enum PxBroadPhaseType : int32_t {
    /// 3-axes sweep-and-prune
    Sap = 0,
    /// Multi box pruning
    Mbp = 1,
    /// Automatic box pruning
    Abp = 2,
    /// Parallel automatic box pruning
    Pabp = 3,
    /// GPU broad phase
    Gpu = 4,
    Last = 5,
};

/// Enum for selecting the friction algorithm used for simulation.
///
/// [`PxFrictionType::ePATCH`] selects the patch friction model which typically leads to the most stable results at low solver iteration counts and is also quite inexpensive, as it uses only
/// up to four scalar solver constraints per pair of touching objects.  The patch friction model is the same basic strong friction algorithm as PhysX 3.2 and before.
///
/// [`PxFrictionType::eONE_DIRECTIONAL`] is a simplification of the Coulomb friction model, in which the friction for a given point of contact is applied in the alternating tangent directions of
/// the contact's normal.  This simplification allows us to reduce the number of iterations required for convergence but is not as accurate as the two directional model.
///
/// [`PxFrictionType::eTWO_DIRECTIONAL`] is identical to the one directional model, but it applies friction in both tangent directions simultaneously.  This hurts convergence a bit so it
/// requires more solver iterations, but is more accurate.  Like the one directional model, it is applied at every contact point, which makes it potentially more expensive
/// than patch friction for scenarios with many contact points.
///
/// [`PxFrictionType::eFRICTION_COUNT`] is the total numer of friction models supported by the SDK.
enum PxFrictionType : int32_t {
    /// Select default patch-friction model.
    Patch = 0,
    /// Select one directional per-contact friction model.
    OneDirectional = 1,
    /// Select two directional per-contact friction model.
    TwoDirectional = 2,
    /// The total number of friction models supported by the SDK.
    FrictionCount = 3,
};

/// Enum for selecting the type of solver used for the simulation.
///
/// [`PxSolverType::ePGS`] selects the iterative sequential impulse solver. This is the same kind of solver used in PhysX 3.4 and earlier releases.
///
/// [`PxSolverType::eTGS`] selects a non linear iterative solver. This kind of solver can lead to improved convergence and handle large mass ratios, long chains and jointed systems better. It is slightly more expensive than the default solver and can introduce more energy to correct joint and contact errors.
enum PxSolverType : int32_t {
    /// Projected Gauss-Seidel iterative solver
    Pgs = 0,
    /// Default Temporal Gauss-Seidel solver
    Tgs = 1,
};

/// flags for configuring properties of the scene
enum PxSceneFlag : int32_t {
    /// Enable Active Actors Notification.
    ///
    /// This flag enables the Active Actor Notification feature for a scene.  This
    /// feature defaults to disabled.  When disabled, the function
    /// PxScene::getActiveActors() will always return a NULL list.
    ///
    /// There may be a performance penalty for enabling the Active Actor Notification, hence this flag should
    /// only be enabled if the application intends to use the feature.
    ///
    /// Default:
    /// False
    EnableActiveActors = 1,
    /// Enables a second broad phase check after integration that makes it possible to prevent objects from tunneling through eachother.
    ///
    /// PxPairFlag::eDETECT_CCD_CONTACT requires this flag to be specified.
    ///
    /// For this feature to be effective for bodies that can move at a significant velocity, the user should raise the flag PxRigidBodyFlag::eENABLE_CCD for them.
    ///
    /// This flag is not mutable, and must be set in PxSceneDesc at scene creation.
    ///
    /// Default:
    /// False
    EnableCcd = 2,
    /// Enables a simplified swept integration strategy, which sacrifices some accuracy for improved performance.
    ///
    /// This simplified swept integration approach makes certain assumptions about the motion of objects that are not made when using a full swept integration.
    /// These assumptions usually hold but there are cases where they could result in incorrect behavior between a set of fast-moving rigid bodies. A key issue is that
    /// fast-moving dynamic objects may tunnel through each-other after a rebound. This will not happen if this mode is disabled. However, this approach will be potentially
    /// faster than a full swept integration because it will perform significantly fewer sweeps in non-trivial scenes involving many fast-moving objects. This approach
    /// should successfully resist objects passing through the static environment.
    ///
    /// PxPairFlag::eDETECT_CCD_CONTACT requires this flag to be specified.
    ///
    /// This scene flag requires eENABLE_CCD to be enabled as well. If it is not, this scene flag will do nothing.
    ///
    /// For this feature to be effective for bodies that can move at a significant velocity, the user should raise the flag PxRigidBodyFlag::eENABLE_CCD for them.
    ///
    /// This flag is not mutable, and must be set in PxSceneDesc at scene creation.
    ///
    /// Default:
    /// False
    DisableCcdResweep = 4,
    /// Enable GJK-based distance collision detection system.
    ///
    /// This flag is not mutable, and must be set in PxSceneDesc at scene creation.
    ///
    /// Default:
    /// true
    EnablePcm = 64,
    /// Disable contact report buffer resize. Once the contact buffer is full, the rest of the contact reports will
    /// not be buffered and sent.
    ///
    /// This flag is not mutable, and must be set in PxSceneDesc at scene creation.
    ///
    /// Default:
    /// false
    DisableContactReportBufferResize = 128,
    /// Disable contact cache.
    ///
    /// Contact caches are used internally to provide faster contact generation. You can disable all contact caches
    /// if memory usage for this feature becomes too high.
    ///
    /// This flag is not mutable, and must be set in PxSceneDesc at scene creation.
    ///
    /// Default:
    /// false
    DisableContactCache = 256,
    /// Require scene-level locking
    ///
    /// When set to true this requires that threads accessing the PxScene use the
    /// multi-threaded lock methods.
    ///
    /// This flag is not mutable, and must be set in PxSceneDesc at scene creation.
    ///
    /// Default:
    /// false
    RequireRwLock = 512,
    /// Enables additional stabilization pass in solver
    ///
    /// When set to true, this enables additional stabilization processing to improve that stability of complex interactions between large numbers of bodies.
    ///
    /// Note that this flag is not mutable and must be set in PxSceneDesc at scene creation. Also, this is an experimental feature which does result in some loss of momentum.
    EnableStabilization = 1024,
    /// Enables average points in contact manifolds
    ///
    /// When set to true, this enables additional contacts to be generated per manifold to represent the average point in a manifold. This can stabilize stacking when only a small
    /// number of solver iterations is used.
    ///
    /// Note that this flag is not mutable and must be set in PxSceneDesc at scene creation.
    EnableAveragePoint = 2048,
    /// Do not report kinematics in list of active actors.
    ///
    /// Since the target pose for kinematics is set by the user, an application can track the activity state directly and use
    /// this flag to avoid that kinematics get added to the list of active actors.
    ///
    /// This flag has only an effect in combination with eENABLE_ACTIVE_ACTORS.
    ///
    /// Default:
    /// false
    ExcludeKinematicsFromActiveActors = 4096,
    /// Do not report kinematics in list of active actors.
    ///
    /// Since the target pose for kinematics is set by the user, an application can track the activity state directly and use
    /// this flag to avoid that kinematics get added to the list of active actors.
    ///
    /// This flag has only an effect in combination with eENABLE_ACTIVE_ACTORS.
    ///
    /// Default:
    /// false
    EnableGpuDynamics = 8192,
    /// Provides improved determinism at the expense of performance.
    ///
    /// By default, PhysX provides limited determinism guarantees. Specifically, PhysX guarantees that the exact scene (same actors created in the same order) and simulated using the same
    /// time-stepping scheme should provide the exact same behaviour.
    ///
    /// However, if additional actors are added to the simulation, this can affect the behaviour of the existing actors in the simulation, even if the set of new actors do not interact with
    /// the existing actors.
    ///
    /// This flag provides an additional level of determinism that guarantees that the simulation will not change if additional actors are added to the simulation, provided those actors do not interfere
    /// with the existing actors in the scene. Determinism is only guaranteed if the actors are inserted in a consistent order each run in a newly-created scene and simulated using a consistent time-stepping
    /// scheme.
    ///
    /// Note that this flag is not mutable and must be set at scene creation.
    ///
    /// Note that enabling this flag can have a negative impact on performance.
    ///
    /// Note that this feature is not currently supported on GPU.
    ///
    /// Default
    /// false
    EnableEnhancedDeterminism = 16384,
    /// Controls processing friction in all solver iterations
    ///
    /// By default, PhysX processes friction only in the final 3 position iterations, and all velocity
    /// iterations. This flag enables friction processing in all position and velocity iterations.
    ///
    /// The default behaviour provides a good trade-off between performance and stability and is aimed
    /// primarily at game development.
    ///
    /// When simulating more complex frictional behaviour, such as grasping of complex geometries with
    /// a robotic manipulator, better results can be achieved by enabling friction in all solver iterations.
    ///
    /// This flag only has effect with the default solver. The TGS solver always performs friction per-iteration.
    EnableFrictionEveryIteration = 32768,
    /// Controls processing friction in all solver iterations
    ///
    /// By default, PhysX processes friction only in the final 3 position iterations, and all velocity
    /// iterations. This flag enables friction processing in all position and velocity iterations.
    ///
    /// The default behaviour provides a good trade-off between performance and stability and is aimed
    /// primarily at game development.
    ///
    /// When simulating more complex frictional behaviour, such as grasping of complex geometries with
    /// a robotic manipulator, better results can be achieved by enabling friction in all solver iterations.
    ///
    /// This flag only has effect with the default solver. The TGS solver always performs friction per-iteration.
    SuppressReadback = 65536,
    /// Controls processing friction in all solver iterations
    ///
    /// By default, PhysX processes friction only in the final 3 position iterations, and all velocity
    /// iterations. This flag enables friction processing in all position and velocity iterations.
    ///
    /// The default behaviour provides a good trade-off between performance and stability and is aimed
    /// primarily at game development.
    ///
    /// When simulating more complex frictional behaviour, such as grasping of complex geometries with
    /// a robotic manipulator, better results can be achieved by enabling friction in all solver iterations.
    ///
    /// This flag only has effect with the default solver. The TGS solver always performs friction per-iteration.
    ForceReadback = 131072,
    /// Controls processing friction in all solver iterations
    ///
    /// By default, PhysX processes friction only in the final 3 position iterations, and all velocity
    /// iterations. This flag enables friction processing in all position and velocity iterations.
    ///
    /// The default behaviour provides a good trade-off between performance and stability and is aimed
    /// primarily at game development.
    ///
    /// When simulating more complex frictional behaviour, such as grasping of complex geometries with
    /// a robotic manipulator, better results can be achieved by enabling friction in all solver iterations.
    ///
    /// This flag only has effect with the default solver. The TGS solver always performs friction per-iteration.
    MutableFlags = 69633,
};

enum PxSceneFlags : uint32_t {
    EnableActiveActors_Bit = 1 << 0,
    EnableCcd_Bit = 1 << 1,
    DisableCcdResweep_Bit = 1 << 2,
    EnablePcm_Bit = 1 << 6,
    DisableContactReportBufferResize_Bit = 1 << 7,
    DisableContactCache_Bit = 1 << 8,
    RequireRwLock_Bit = 1 << 9,
    EnableStabilization_Bit = 1 << 10,
    EnableAveragePoint_Bit = 1 << 11,
    ExcludeKinematicsFromActiveActors_Bit = 1 << 12,
    EnableGpuDynamics_Bit = 1 << 13,
    EnableEnhancedDeterminism_Bit = 1 << 14,
    EnableFrictionEveryIteration_Bit = 1 << 15,
    SuppressReadback_Bit = 1 << 16,
    ForceReadback_Bit = 1 << 17,
    MutableFlags_Bit = EnableActiveActors | ExcludeKinematicsFromActiveActors | SuppressReadback,
};

/// Debug visualization parameters.
///
/// [`PxVisualizationParameter::eSCALE`] is the master switch for enabling visualization, please read the corresponding documentation
/// for further details.
enum PxVisualizationParameter : int32_t {
    /// This overall visualization scale gets multiplied with the individual scales. Setting to zero ignores all visualizations. Default is 0.
    ///
    /// The below settings permit the debug visualization of various simulation properties.
    /// The setting is either zero, in which case the property is not drawn. Otherwise it is a scaling factor
    /// that determines the size of the visualization widgets.
    ///
    /// Only objects for which visualization is turned on using setFlag(eVISUALIZATION) are visualized (see [`PxActorFlag::eVISUALIZATION`], #PxShapeFlag::eVISUALIZATION, ...).
    /// Contacts are visualized if they involve a body which is being visualized.
    /// Default is 0.
    ///
    /// Notes:
    /// - to see any visualization, you have to set PxVisualizationParameter::eSCALE to nonzero first.
    /// - the scale factor has been introduced because it's difficult (if not impossible) to come up with a
    /// good scale for 3D vectors. Normals are normalized and their length is always 1. But it doesn't mean
    /// we should render a line of length 1. Depending on your objects/scene, this might be completely invisible
    /// or extremely huge. That's why the scale factor is here, to let you tune the length until it's ok in
    /// your scene.
    /// - however, things like collision shapes aren't ambiguous. They are clearly defined for example by the
    /// triangles
    /// &
    /// polygons themselves, and there's no point in scaling that. So the visualization widgets
    /// are only scaled when it makes sense.
    ///
    /// Range:
    /// [0, PX_MAX_F32)
    /// Default:
    /// 0
    Scale = 0,
    /// Visualize the world axes.
    WorldAxes = 1,
    /// Visualize a bodies axes.
    BodyAxes = 2,
    /// Visualize a body's mass axes.
    ///
    /// This visualization is also useful for visualizing the sleep state of bodies. Sleeping bodies are drawn in
    /// black, while awake bodies are drawn in white. If the body is sleeping and part of a sleeping group, it is
    /// drawn in red.
    BodyMassAxes = 3,
    /// Visualize the bodies linear velocity.
    BodyLinVelocity = 4,
    /// Visualize the bodies angular velocity.
    BodyAngVelocity = 5,
    /// Visualize contact points. Will enable contact information.
    ContactPoint = 6,
    /// Visualize contact normals. Will enable contact information.
    ContactNormal = 7,
    /// Visualize contact errors. Will enable contact information.
    ContactError = 8,
    /// Visualize Contact forces. Will enable contact information.
    ContactForce = 9,
    /// Visualize actor axes.
    ActorAxes = 10,
    /// Visualize bounds (AABBs in world space)
    CollisionAabbs = 11,
    /// Shape visualization
    CollisionShapes = 12,
    /// Shape axis visualization
    CollisionAxes = 13,
    /// Compound visualization (compound AABBs in world space)
    CollisionCompounds = 14,
    /// Mesh
    /// &
    /// convex face normals
    CollisionFnormals = 15,
    /// Active edges for meshes
    CollisionEdges = 16,
    /// Static pruning structures
    CollisionStatic = 17,
    /// Dynamic pruning structures
    CollisionDynamic = 18,
    /// Joint local axes
    JointLocalFrames = 19,
    /// Joint limits
    JointLimits = 20,
    /// Visualize culling box
    CullBox = 21,
    /// MBP regions
    MbpRegions = 22,
    /// Renders the simulation mesh instead of the collision mesh (only available for tetmeshes)
    SimulationMesh = 23,
    /// Renders the SDF of a mesh instead of the collision mesh (only available for triangle meshes with SDFs)
    Sdf = 24,
    /// This is not a parameter, it just records the current number of parameters (as maximum(PxVisualizationParameter)+1) for use in loops.
    NumValues = 25,
    /// This is not a parameter, it just records the current number of parameters (as maximum(PxVisualizationParameter)+1) for use in loops.
    ForceDword = 2147483647,
};

/// Different types of rigid body collision pair statistics.
enum RbPairStatsType : int32_t {
    /// Shape pairs processed as discrete contact pairs for the current simulation step.
    DiscreteContactPairs = 0,
    /// Shape pairs processed as swept integration pairs for the current simulation step.
    ///
    /// Counts the pairs for which special CCD (continuous collision detection) work was actually done and NOT the number of pairs which were configured for CCD.
    /// Furthermore, there can be multiple CCD passes and all processed pairs of all passes are summed up, hence the number can be larger than the amount of pairs which have been configured for CCD.
    CcdPairs = 1,
    /// Shape pairs processed with user contact modification enabled for the current simulation step.
    ModifiedContactPairs = 2,
    /// Trigger shape pairs processed for the current simulation step.
    TriggerPairs = 3,
};

/// These flags determine what data is read or written to the gpu softbody.
enum PxSoftBodyDataFlag : int32_t {
    /// The collision mesh tetrahedron indices (quadruples of int32)
    TetIndices = 0,
    /// The collision mesh cauchy stress tensors (float 3x3 matrices)
    TetStress = 1,
    /// The collision mesh tetrahedron von Mises stress (float scalar)
    TetStresscoeff = 2,
    /// The collision mesh tetrahedron rest poses (float 3x3 matrices)
    TetRestPoses = 3,
    /// The collision mesh tetrahedron orientations (quaternions, quadruples of float)
    TetRotations = 4,
    /// The collision mesh vertex positions and their inverted mass in the 4th component (quadruples of float)
    TetPositionInvMass = 5,
    /// The simulation mesh tetrahedron indices (quadruples of int32)
    SimTetIndices = 6,
    /// The simulation mesh vertex velocities and their inverted mass in the 4th component (quadruples of float)
    SimVelocityInvMass = 7,
    /// The simulation mesh vertex positions and their inverted mass in the 4th component (quadruples of float)
    SimPositionInvMass = 8,
    /// The simulation mesh kinematic target positions
    SimKinematicTarget = 9,
};

/// Identifies input and output buffers for PxHairSystem
enum PxHairSystemData : int32_t {
    /// No data specified
    None = 0,
    /// Specifies the position (first 3 floats) and inverse mass (last float) data (array of PxVec4 * max number of vertices)
    PositionInvmass = 1,
    /// Specifies the velocity (first 3 floats) data (array of PxVec4 * max number of vertices)
    Velocity = 2,
    /// Specifies everything
    All = 3,
};

enum PxHairSystemDataFlags : uint32_t {
    PositionInvmass_Bit = 1 << 0,
    Velocity_Bit = 1 << 1,
    All_Bit = PositionInvmass | Velocity,
};

/// Binary settings for hair system simulation
enum PxHairSystemFlag : int32_t {
    /// Determines if self-collision between hair vertices is ignored
    DisableSelfCollision = 1,
    /// Determines if collision between hair and external bodies is ignored
    DisableExternalCollision = 2,
    /// Determines if attachment constraint is also felt by body to which the hair is attached
    DisableTwosidedAttachment = 4,
};

enum PxHairSystemFlags : uint32_t {
    DisableSelfCollision_Bit = 1 << 0,
    DisableExternalCollision_Bit = 1 << 1,
    DisableTwosidedAttachment_Bit = 1 << 2,
};

/// Identifies each type of information for retrieving from actor.
enum PxActorCacheFlag : int32_t {
    ActorData = 1,
    Force = 4,
    Torque = 8,
};

enum PxActorCacheFlags : uint16_t {
    ActorData_Bit = 1 << 0,
    Force_Bit = 1 << 2,
    Torque_Bit = 1 << 3,
};

/// PVD scene Flags. They are disabled by default, and only works if PxPvdInstrumentationFlag::eDEBUG is set.
enum PxPvdSceneFlag : int32_t {
    TransmitContacts = 1,
    /// Transmits contact stream to PVD.
    TransmitScenequeries = 2,
    /// Transmits scene query stream to PVD.
    TransmitConstraints = 4,
};

enum PxPvdSceneFlags : uint8_t {
    TransmitContacts_Bit = 1 << 0,
    TransmitScenequeries_Bit = 1 << 1,
    TransmitConstraints_Bit = 1 << 2,
};

/// Identifies each type of actor for retrieving actors from a scene.
///
/// [`PxArticulationLink`] objects are not supported. Use the #PxArticulationReducedCoordinate object to retrieve all its links.
enum PxActorTypeFlag : int32_t {
    /// A static rigid body
    RigidStatic = 1,
    /// A dynamic rigid body
    RigidDynamic = 2,
};

enum PxActorTypeFlags : uint16_t {
    RigidStatic_Bit = 1 << 0,
    RigidDynamic_Bit = 1 << 1,
};

/// Extra data item types for contact pairs.
enum PxContactPairExtraDataType : int32_t {
    /// see [`PxContactPairVelocity`]
    PreSolverVelocity = 0,
    /// see [`PxContactPairVelocity`]
    PostSolverVelocity = 1,
    /// see [`PxContactPairPose`]
    ContactEventPose = 2,
    /// see [`PxContactPairIndex`]
    ContactPairIndex = 3,
};

/// Collection of flags providing information on contact report pairs.
enum PxContactPairHeaderFlag : int32_t {
    /// The actor with index 0 has been removed from the scene.
    RemovedActor0 = 1,
    /// The actor with index 1 has been removed from the scene.
    RemovedActor1 = 2,
};

enum PxContactPairHeaderFlags : uint16_t {
    RemovedActor0_Bit = 1 << 0,
    RemovedActor1_Bit = 1 << 1,
};

/// Collection of flags providing information on contact report pairs.
enum PxContactPairFlag : int32_t {
    /// The shape with index 0 has been removed from the actor/scene.
    RemovedShape0 = 1,
    /// The shape with index 1 has been removed from the actor/scene.
    RemovedShape1 = 2,
    /// First actor pair contact.
    ///
    /// The provided shape pair marks the first contact between the two actors, no other shape pair has been touching prior to the current simulation frame.
    ///
    /// : This info is only available if [`PxPairFlag::eNOTIFY_TOUCH_FOUND`] has been declared for the pair.
    ActorPairHasFirstTouch = 4,
    /// All contact between the actor pair was lost.
    ///
    /// All contact between the two actors has been lost, no shape pairs remain touching after the current simulation frame.
    ActorPairLostTouch = 8,
    /// Internal flag, used by [`PxContactPair`].extractContacts()
    ///
    /// The applied contact impulses are provided for every contact point.
    /// This is the case if [`PxPairFlag::eSOLVE_CONTACT`] has been set for the pair.
    InternalHasImpulses = 16,
    /// Internal flag, used by [`PxContactPair`].extractContacts()
    ///
    /// The provided contact point information is flipped with regards to the shapes of the contact pair. This mainly concerns the order of the internal triangle indices.
    InternalContactsAreFlipped = 32,
};

enum PxContactPairFlags : uint16_t {
    RemovedShape0_Bit = 1 << 0,
    RemovedShape1_Bit = 1 << 1,
    ActorPairHasFirstTouch_Bit = 1 << 2,
    ActorPairLostTouch_Bit = 1 << 3,
    InternalHasImpulses_Bit = 1 << 4,
    InternalContactsAreFlipped_Bit = 1 << 5,
};

/// Collection of flags providing information on trigger report pairs.
enum PxTriggerPairFlag : int32_t {
    /// The trigger shape has been removed from the actor/scene.
    RemovedShapeTrigger = 1,
    /// The shape causing the trigger event has been removed from the actor/scene.
    RemovedShapeOther = 2,
    /// For internal use only.
    NextFree = 4,
};

enum PxTriggerPairFlags : uint8_t {
    RemovedShapeTrigger_Bit = 1 << 0,
    RemovedShapeOther_Bit = 1 << 1,
    NextFree_Bit = 1 << 2,
};

/// Identifies input and output buffers for PxSoftBody.
enum PxSoftBodyData : int32_t {
    None = 0,
    /// Flag to request access to the collision mesh's positions; read only
    PositionInvmass = 1,
    /// Flag to request access to the simulation mesh's positions and inverse masses
    SimPositionInvmass = 4,
    /// Flag to request access to the simulation mesh's velocities and inverse masses
    SimVelocity = 8,
    /// Flag to request access to the simulation mesh's kinematic target position
    SimKinematicTarget = 16,
    All = 29,
};

enum PxSoftBodyDataFlags : uint32_t {
    PositionInvmass_Bit = 1 << 0,
    SimPositionInvmass_Bit = 1 << 2,
    SimVelocity_Bit = 1 << 3,
    SimKinematicTarget_Bit = 1 << 4,
    All_Bit = PositionInvmass | SimPositionInvmass | SimVelocity | SimKinematicTarget,
};

/// Flags to enable or disable special modes of a SoftBody
enum PxSoftBodyFlag : int32_t {
    /// Determines if self collision will be detected and resolved
    DisableSelfCollision = 1,
    /// Enables computation of a Cauchy stress tensor for every tetrahedron in the simulation mesh. The tensors can be accessed through the softbody direct API
    ComputeStressTensor = 2,
    /// Enables support for continuous collision detection
    EnableCcd = 4,
    /// Enable debug rendering to display the simulation mesh
    DisplaySimMesh = 8,
    /// Enables support for kinematic motion of the collision and simulation mesh.
    Kinematic = 16,
    /// Enables partially kinematic motion of the collisios and simulation mesh.
    PartiallyKinematic = 32,
};

enum PxSoftBodyFlags : uint32_t {
    DisableSelfCollision_Bit = 1 << 0,
    ComputeStressTensor_Bit = 1 << 1,
    EnableCcd_Bit = 1 << 2,
    DisplaySimMesh_Bit = 1 << 3,
    Kinematic_Bit = 1 << 4,
    PartiallyKinematic_Bit = 1 << 5,
};

/// The type of controller, eg box, sphere or capsule.
enum PxControllerShapeType : int32_t {
    /// A box controller.
    Box = 0,
    /// A capsule controller
    Capsule = 1,
    /// A capsule controller
    ForceDword = 2147483647,
};

/// specifies how a CCT interacts with non-walkable parts.
///
/// This is only used when slopeLimit is non zero. It is currently enabled for static actors only, and not supported for spheres or capsules.
enum PxControllerNonWalkableMode : int32_t {
    /// Stops character from climbing up non-walkable slopes, but doesn't move it otherwise
    PreventClimbing = 0,
    /// Stops character from climbing up non-walkable slopes, and forces it to slide down those slopes
    PreventClimbingAndForceSliding = 1,
};

/// specifies which sides a character is colliding with.
enum PxControllerCollisionFlag : int32_t {
    /// Character is colliding to the sides.
    CollisionSides = 1,
    /// Character has collision above.
    CollisionUp = 2,
    /// Character has collision below.
    CollisionDown = 4,
};

enum PxControllerCollisionFlags : uint8_t {
    CollisionSides_Bit = 1 << 0,
    CollisionUp_Bit = 1 << 1,
    CollisionDown_Bit = 1 << 2,
};

enum PxCapsuleClimbingMode : int32_t {
    /// Standard mode, let the capsule climb over surfaces according to impact normal
    Easy = 0,
    /// Constrained mode, try to limit climbing according to the step offset
    Constrained = 1,
    Last = 2,
};

/// specifies controller behavior
enum PxControllerBehaviorFlag : int32_t {
    /// Controller can ride on touched object (i.e. when this touched object is moving horizontally).
    ///
    /// The CCT vs. CCT case is not supported.
    CctCanRideOnObject = 1,
    /// Controller should slide on touched object
    CctSlide = 2,
    /// Disable all code dealing with controllers riding on objects, let users define it outside of the SDK.
    CctUserDefinedRide = 4,
};

enum PxControllerBehaviorFlags : uint8_t {
    CctCanRideOnObject_Bit = 1 << 0,
    CctSlide_Bit = 1 << 1,
    CctUserDefinedRide_Bit = 1 << 2,
};

/// specifies debug-rendering flags
enum PxControllerDebugRenderFlag : uint32_t {
    /// Temporal bounding volume around controllers
    TemporalBv = 1,
    /// Cached bounding volume around controllers
    CachedBv = 2,
    /// User-defined obstacles
    Obstacles = 4,
    None = 0,
    All = 4294967295,
};

enum PxControllerDebugRenderFlags : uint32_t {
    TemporalBv_Bit = 1 << 0,
    CachedBv_Bit = 1 << 1,
    Obstacles_Bit = 1 << 2,
    All_Bit = TemporalBv | CachedBv | Obstacles,
};

/// Defines the number of bits per subgrid pixel
enum PxSdfBitsPerSubgridPixel : int32_t {
    /// 8 bit per subgrid pixel (values will be stored as normalized integers)
    E8BitPerPixel = 1,
    /// 16 bit per subgrid pixel (values will be stored as normalized integers)
    E16BitPerPixel = 2,
    /// 32 bit per subgrid pixel (values will be stored as floats in world scale units)
    E32BitPerPixel = 4,
};

/// Flags which describe the format and behavior of a convex mesh.
enum PxConvexFlag : int32_t {
    /// Denotes the use of 16-bit vertex indices in PxConvexMeshDesc::triangles or PxConvexMeshDesc::polygons.
    /// (otherwise, 32-bit indices are assumed)
    E16BitIndices = 1,
    /// Automatically recomputes the hull from the vertices. If this flag is not set, you must provide the entire geometry manually.
    ///
    /// There are two different algorithms for hull computation, please see PxConvexMeshCookingType.
    ComputeConvex = 2,
    /// Checks and removes almost zero-area triangles during convex hull computation.
    /// The rejected area size is specified in PxCookingParams::areaTestEpsilon
    ///
    /// This flag is only used in combination with eCOMPUTE_CONVEX.
    CheckZeroAreaTriangles = 4,
    /// Quantizes the input vertices using the k-means clustering
    ///
    /// The input vertices are quantized to PxConvexMeshDesc::quantizedCount
    /// see http://en.wikipedia.org/wiki/K-means_clustering
    QuantizeInput = 8,
    /// Disables the convex mesh validation to speed-up hull creation. Please use separate validation
    /// function in checked/debug builds. Creating a convex mesh with invalid input data without prior validation
    /// may result in undefined behavior.
    DisableMeshValidation = 16,
    /// Enables plane shifting vertex limit algorithm.
    ///
    /// Plane shifting is an alternative algorithm for the case when the computed hull has more vertices
    /// than the specified vertex limit.
    ///
    /// The default algorithm computes the full hull, and an OBB around the input vertices. This OBB is then sliced
    /// with the hull planes until the vertex limit is reached.The default algorithm requires the vertex limit
    /// to be set to at least 8, and typically produces results that are much better quality than are produced
    /// by plane shifting.
    ///
    /// When plane shifting is enabled, the hull computation stops when vertex limit is reached. The hull planes
    /// are then shifted to contain all input vertices, and the new plane intersection points are then used to
    /// generate the final hull with the given vertex limit.Plane shifting may produce sharp edges to vertices
    /// very far away from the input cloud, and does not guarantee that all input vertices are inside the resulting
    /// hull.However, it can be used with a vertex limit as low as 4.
    PlaneShifting = 32,
    /// Inertia tensor computation is faster using SIMD code, but the precision is lower, which may result
    /// in incorrect inertia for very thin hulls.
    FastInertiaComputation = 64,
    /// Convex hulls are created with respect to GPU simulation limitations. Vertex limit and polygon limit
    /// is set to 64 and vertex limit per face is internally set to 32.
    ///
    /// Can be used only with eCOMPUTE_CONVEX flag.
    GpuCompatible = 128,
    /// Convex hull input vertices are shifted to be around origin to provide better computation stability.
    /// It is recommended to provide input vertices around the origin, otherwise use this flag to improve
    /// numerical stability.
    ///
    /// Is used only with eCOMPUTE_CONVEX flag.
    ShiftVertices = 256,
};

enum PxConvexFlags : uint16_t {
    E16BitIndices_Bit = 1 << 0,
    ComputeConvex_Bit = 1 << 1,
    CheckZeroAreaTriangles_Bit = 1 << 2,
    QuantizeInput_Bit = 1 << 3,
    DisableMeshValidation_Bit = 1 << 4,
    PlaneShifting_Bit = 1 << 5,
    FastInertiaComputation_Bit = 1 << 6,
    GpuCompatible_Bit = 1 << 7,
    ShiftVertices_Bit = 1 << 8,
};

/// Defines the tetrahedron structure of a mesh.
enum PxMeshFormat : int32_t {
    /// Normal tetmesh with arbitrary tetrahedra
    TetMesh = 0,
    /// 6 tetrahedra in a row will form a hexahedron
    HexMesh = 1,
};

/// Desired build strategy for PxMeshMidPhase::eBVH34
enum PxBVH34BuildStrategy : int32_t {
    /// Fast build strategy. Fast build speed, good runtime performance in most cases. Recommended for runtime mesh cooking.
    Fast = 0,
    /// Default build strategy. Medium build speed, good runtime performance in all cases.
    Default = 1,
    /// SAH build strategy. Slower builds, slightly improved runtime performance in some cases.
    Sah = 2,
    Last = 3,
};

/// Result from convex cooking.
enum PxConvexMeshCookingResult : int32_t {
    /// Convex mesh cooking succeeded.
    Success = 0,
    /// Convex mesh cooking failed, algorithm couldn't find 4 initial vertices without a small triangle.
    ZeroAreaTestFailed = 1,
    /// Convex mesh cooking succeeded, but the algorithm has reached the 255 polygons limit.
    /// The produced hull does not contain all input vertices. Try to simplify the input vertices
    /// or try to use the eINFLATE_CONVEX or the eQUANTIZE_INPUT flags.
    PolygonsLimitReached = 2,
    /// Something unrecoverable happened. Check the error stream to find out what.
    Failure = 3,
};

/// Enumeration for convex mesh cooking algorithms.
enum PxConvexMeshCookingType : int32_t {
    /// The Quickhull algorithm constructs the hull from the given input points. The resulting hull
    /// will only contain a subset of the input points.
    Quickhull = 0,
};

/// Result from triangle mesh cooking
enum PxTriangleMeshCookingResult : int32_t {
    /// Everything is A-OK.
    Success = 0,
    /// a triangle is too large for well-conditioned results. Tessellate the mesh for better behavior, see the user guide section on cooking for more details.
    LargeTriangle = 1,
    /// Something unrecoverable happened. Check the error stream to find out what.
    Failure = 2,
};

/// Enum for the set of mesh pre-processing parameters.
enum PxMeshPreprocessingFlag : int32_t {
    /// When set, mesh welding is performed. See PxCookingParams::meshWeldTolerance. Clean mesh must be enabled.
    WeldVertices = 1,
    /// When set, mesh cleaning is disabled. This makes cooking faster.
    ///
    /// When clean mesh is not performed, mesh welding is also not performed.
    ///
    /// It is recommended to use only meshes that passed during validateTriangleMesh.
    DisableCleanMesh = 2,
    /// When set, active edges are set for each triangle edge. This makes cooking faster but slow up contact generation.
    DisableActiveEdgesPrecompute = 4,
    /// When set, 32-bit indices will always be created regardless of triangle count.
    ///
    /// By default mesh will be created with 16-bit indices for triangle count
    /// <
    /// = 0xFFFF and 32-bit otherwise.
    Force32bitIndices = 8,
    /// When set, a list of triangles will be created for each associated vertex in the mesh
    EnableVertMapping = 16,
    /// When set, inertia tensor is calculated for the mesh
    EnableInertia = 32,
};

enum PxMeshPreprocessingFlags : uint32_t {
    WeldVertices_Bit = 1 << 0,
    DisableCleanMesh_Bit = 1 << 1,
    DisableActiveEdgesPrecompute_Bit = 1 << 2,
    Force32bitIndices_Bit = 1 << 3,
    EnableVertMapping_Bit = 1 << 4,
    EnableInertia_Bit = 1 << 5,
};

/// Unique identifiers for extensions classes which implement a constraint based on PxConstraint.
///
/// Users which want to create their own custom constraint types should choose an ID larger or equal to eNEXT_FREE_ID
/// and not eINVALID_ID.
enum PxConstraintExtIDs : int32_t {
    Joint = 0,
    VehicleSuspLimitDeprecated = 1,
    VehicleStickyTyreDeprecated = 2,
    VehicleJoint = 3,
    NextFreeId = 4,
    InvalidId = 2147483647,
};

/// an enumeration of PhysX' built-in joint types
enum PxJointConcreteType : int32_t {
    Spherical = 256,
    Revolute = 257,
    Prismatic = 258,
    Fixed = 259,
    Distance = 260,
    D6 = 261,
    Contact = 262,
    Gear = 263,
    RackAndPinion = 264,
    Last = 265,
};

/// an enumeration for specifying one or other of the actors referenced by a joint
enum PxJointActorIndex : int32_t {
    Actor0 = 0,
    Actor1 = 1,
    Count = 2,
};

/// flags for configuring the drive of a PxDistanceJoint
enum PxDistanceJointFlag : int32_t {
    MaxDistanceEnabled = 2,
    MinDistanceEnabled = 4,
    SpringEnabled = 8,
};

enum PxDistanceJointFlags : uint16_t {
    MaxDistanceEnabled_Bit = 1 << 1,
    MinDistanceEnabled_Bit = 1 << 2,
    SpringEnabled_Bit = 1 << 3,
};

/// Flags specific to the prismatic joint.
enum PxPrismaticJointFlag : int32_t {
    LimitEnabled = 2,
};

enum PxPrismaticJointFlags : uint16_t {
    LimitEnabled_Bit = 1 << 1,
};

/// Flags specific to the Revolute Joint.
enum PxRevoluteJointFlag : int32_t {
    /// enable the limit
    LimitEnabled = 1,
    /// enable the drive
    DriveEnabled = 2,
    /// if the existing velocity is beyond the drive velocity, do not add force
    DriveFreespin = 4,
};

enum PxRevoluteJointFlags : uint16_t {
    LimitEnabled_Bit = 1 << 0,
    DriveEnabled_Bit = 1 << 1,
    DriveFreespin_Bit = 1 << 2,
};

/// Flags specific to the spherical joint.
enum PxSphericalJointFlag : int32_t {
    /// the cone limit for the spherical joint is enabled
    LimitEnabled = 2,
};

enum PxSphericalJointFlags : uint16_t {
    LimitEnabled_Bit = 1 << 1,
};

/// Used to specify one of the degrees of freedom of  a D6 joint.
enum PxD6Axis : int32_t {
    /// motion along the X axis
    X = 0,
    /// motion along the Y axis
    Y = 1,
    /// motion along the Z axis
    Z = 2,
    /// motion around the X axis
    Twist = 3,
    /// motion around the Y axis
    Swing1 = 4,
    /// motion around the Z axis
    Swing2 = 5,
    Count = 6,
};

/// Used to specify the range of motions allowed for a degree of freedom in a D6 joint.
enum PxD6Motion : int32_t {
    /// The DOF is locked, it does not allow relative motion.
    Locked = 0,
    /// The DOF is limited, it only allows motion within a specific range.
    Limited = 1,
    /// The DOF is free and has its full range of motion.
    Free = 2,
};

/// Used to specify which axes of a D6 joint are driven.
///
/// Each drive is an implicit force-limited damped spring:
///
/// force = spring * (target position - position) + damping * (targetVelocity - velocity)
///
/// Alternatively, the spring may be configured to generate a specified acceleration instead of a force.
///
/// A linear axis is affected by drive only if the corresponding drive flag is set. There are two possible models
/// for angular drive: swing/twist, which may be used to drive one or more angular degrees of freedom, or slerp,
/// which may only be used to drive all three angular degrees simultaneously.
enum PxD6Drive : int32_t {
    /// drive along the X-axis
    X = 0,
    /// drive along the Y-axis
    Y = 1,
    /// drive along the Z-axis
    Z = 2,
    /// drive of displacement from the X-axis
    Swing = 3,
    /// drive of the displacement around the X-axis
    Twist = 4,
    /// drive of all three angular degrees along a SLERP-path
    Slerp = 5,
    Count = 6,
};

/// flags for configuring the drive model of a PxD6Joint
enum PxD6JointDriveFlag : int32_t {
    /// drive spring is for the acceleration at the joint (rather than the force)
    Acceleration = 1,
};

enum PxD6JointDriveFlags : uint32_t {
    Acceleration_Bit = 1 << 0,
};

/// Collision filtering operations.
enum PxFilterOp : int32_t {
    PxFilteropAnd = 0,
    PxFilteropOr = 1,
    PxFilteropXor = 2,
    PxFilteropNand = 3,
    PxFilteropNor = 4,
    PxFilteropNxor = 5,
    PxFilteropSwapAnd = 6,
};

/// If a thread ends up waiting for work it will find itself in a spin-wait loop until work becomes available.
/// Three strategies are available to limit wasted cycles.
/// The strategies are as follows:
/// a) wait until a work task signals the end of the spin-wait period.
/// b) yield the thread by providing a hint to reschedule thread execution, thereby allowing other threads to run.
/// c) yield the processor by informing it that it is waiting for work and requesting it to more efficiently use compute resources.
enum PxDefaultCpuDispatcherWaitForWorkMode : int32_t {
    WaitForWork = 0,
    YieldThread = 1,
    YieldProcessor = 2,
};

enum PxBatchQueryStatus : int32_t {
    /// This is the initial state before a query starts.
    Pending = 0,
    /// The query is finished; results have been written into the result and hit buffers.
    Success = 1,
    /// The query results were incomplete due to touch hit buffer overflow. Blocking hit is still correct.
    Overflow = 2,
};

/// types of instrumentation that PVD can do.
enum PxPvdInstrumentationFlag : int32_t {
    /// Send debugging information to PVD.
    ///
    /// This information is the actual object data of the rigid statics, shapes,
    /// articulations, etc.  Sending this information has a noticeable impact on
    /// performance and thus this flag should not be set if you want an accurate
    /// performance profile.
    Debug = 1,
    /// Send profile information to PVD.
    ///
    /// This information populates PVD's profile view.  It has (at this time) negligible
    /// cost compared to Debug information and makes PVD *much* more useful so it is quite
    /// highly recommended.
    ///
    /// This flag works together with a PxCreatePhysics parameter.
    /// Using it allows the SDK to send profile events to PVD.
    Profile = 2,
    /// Send memory information to PVD.
    ///
    /// The PVD sdk side hooks into the Foundation memory controller and listens to
    /// allocation/deallocation events.  This has a noticable hit on the first frame,
    /// however, this data is somewhat compressed and the PhysX SDK doesn't allocate much
    /// once it hits a steady state.  This information also has a fairly negligible
    /// impact and thus is also highly recommended.
    ///
    /// This flag works together with a PxCreatePhysics parameter,
    /// trackOutstandingAllocations.  Using both of them together allows users to have
    /// an accurate view of the overall memory usage of the simulation at the cost of
    /// a hashtable lookup per allocation/deallocation.  Again, PhysX makes a best effort
    /// attempt not to allocate or deallocate during simulation so this hashtable lookup
    /// tends to have no effect past the first frame.
    ///
    /// Sending memory information without tracking outstanding allocations means that
    /// PVD will accurate information about the state of the memory system before the
    /// actual connection happened.
    Memory = 4,
    /// Send memory information to PVD.
    ///
    /// The PVD sdk side hooks into the Foundation memory controller and listens to
    /// allocation/deallocation events.  This has a noticable hit on the first frame,
    /// however, this data is somewhat compressed and the PhysX SDK doesn't allocate much
    /// once it hits a steady state.  This information also has a fairly negligible
    /// impact and thus is also highly recommended.
    ///
    /// This flag works together with a PxCreatePhysics parameter,
    /// trackOutstandingAllocations.  Using both of them together allows users to have
    /// an accurate view of the overall memory usage of the simulation at the cost of
    /// a hashtable lookup per allocation/deallocation.  Again, PhysX makes a best effort
    /// attempt not to allocate or deallocate during simulation so this hashtable lookup
    /// tends to have no effect past the first frame.
    ///
    /// Sending memory information without tracking outstanding allocations means that
    /// PVD will accurate information about the state of the memory system before the
    /// actual connection happened.
    All = 7,
};

enum PxPvdInstrumentationFlags : uint8_t {
    Debug_Bit = 1 << 0,
    Profile_Bit = 1 << 1,
    Memory_Bit = 1 << 2,
    All_Bit = Debug | Profile | Memory,
};
using namespace physx;
#include "structgen_out.hpp"

static_assert(sizeof(physx::PxAllocatorCallback) == sizeof(physx_PxAllocatorCallback), "POD wrapper for `physx::PxAllocatorCallback` has incorrect size");
static_assert(sizeof(physx::PxAssertHandler) == sizeof(physx_PxAssertHandler), "POD wrapper for `physx::PxAssertHandler` has incorrect size");
static_assert(sizeof(physx::PxFoundation) == sizeof(physx_PxFoundation), "POD wrapper for `physx::PxFoundation` has incorrect size");
static_assert(sizeof(physx::PxAllocator) == sizeof(physx_PxAllocator), "POD wrapper for `physx::PxAllocator` has incorrect size");
static_assert(sizeof(physx::PxRawAllocator) == sizeof(physx_PxRawAllocator), "POD wrapper for `physx::PxRawAllocator` has incorrect size");
static_assert(sizeof(physx::PxVirtualAllocatorCallback) == sizeof(physx_PxVirtualAllocatorCallback), "POD wrapper for `physx::PxVirtualAllocatorCallback` has incorrect size");
static_assert(sizeof(physx::PxVirtualAllocator) == sizeof(physx_PxVirtualAllocator), "POD wrapper for `physx::PxVirtualAllocator` has incorrect size");
static_assert(sizeof(physx::PxUserAllocated) == sizeof(physx_PxUserAllocated), "POD wrapper for `physx::PxUserAllocated` has incorrect size");
static_assert(sizeof(physx::PxTempAllocatorChunk) == sizeof(physx_PxTempAllocatorChunk), "POD wrapper for `physx::PxTempAllocatorChunk` has incorrect size");
static_assert(sizeof(physx::PxTempAllocator) == sizeof(physx_PxTempAllocator), "POD wrapper for `physx::PxTempAllocator` has incorrect size");
static_assert(sizeof(physx::PxBitAndByte) == sizeof(physx_PxBitAndByte), "POD wrapper for `physx::PxBitAndByte` has incorrect size");
static_assert(sizeof(physx::PxBitMap) == sizeof(physx_PxBitMap), "POD wrapper for `physx::PxBitMap` has incorrect size");
static_assert(sizeof(physx::PxVec3) == sizeof(physx_PxVec3), "POD wrapper for `physx::PxVec3` has incorrect size");
static_assert(sizeof(physx::PxVec3Padded) == sizeof(physx_PxVec3Padded), "POD wrapper for `physx::PxVec3Padded` has incorrect size");
static_assert(sizeof(physx::PxQuat) == sizeof(physx_PxQuat), "POD wrapper for `physx::PxQuat` has incorrect size");
static_assert(sizeof(physx::PxTransform) == sizeof(physx_PxTransform), "POD wrapper for `physx::PxTransform` has incorrect size");
static_assert(sizeof(physx::PxTransformPadded) == sizeof(physx_PxTransformPadded), "POD wrapper for `physx::PxTransformPadded` has incorrect size");
static_assert(sizeof(physx::PxMat33) == sizeof(physx_PxMat33), "POD wrapper for `physx::PxMat33` has incorrect size");
static_assert(sizeof(physx::PxBounds3) == sizeof(physx_PxBounds3), "POD wrapper for `physx::PxBounds3` has incorrect size");
static_assert(sizeof(physx::PxErrorCallback) == sizeof(physx_PxErrorCallback), "POD wrapper for `physx::PxErrorCallback` has incorrect size");
static_assert(sizeof(physx::PxAllocationListener) == sizeof(physx_PxAllocationListener), "POD wrapper for `physx::PxAllocationListener` has incorrect size");
static_assert(sizeof(physx::PxBroadcastingAllocator) == sizeof(physx_PxBroadcastingAllocator), "POD wrapper for `physx::PxBroadcastingAllocator` has incorrect size");
static_assert(sizeof(physx::PxBroadcastingErrorCallback) == sizeof(physx_PxBroadcastingErrorCallback), "POD wrapper for `physx::PxBroadcastingErrorCallback` has incorrect size");
static_assert(sizeof(physx::PxInputStream) == sizeof(physx_PxInputStream), "POD wrapper for `physx::PxInputStream` has incorrect size");
static_assert(sizeof(physx::PxInputData) == sizeof(physx_PxInputData), "POD wrapper for `physx::PxInputData` has incorrect size");
static_assert(sizeof(physx::PxOutputStream) == sizeof(physx_PxOutputStream), "POD wrapper for `physx::PxOutputStream` has incorrect size");
static_assert(sizeof(physx::PxVec4) == sizeof(physx_PxVec4), "POD wrapper for `physx::PxVec4` has incorrect size");
static_assert(sizeof(physx::PxMat44) == sizeof(physx_PxMat44), "POD wrapper for `physx::PxMat44` has incorrect size");
static_assert(sizeof(physx::PxPlane) == sizeof(physx_PxPlane), "POD wrapper for `physx::PxPlane` has incorrect size");
static_assert(sizeof(physx::Interpolation) == sizeof(physx_Interpolation), "POD wrapper for `physx::Interpolation` has incorrect size");
static_assert(sizeof(physx::PxMutexImpl) == sizeof(physx_PxMutexImpl), "POD wrapper for `physx::PxMutexImpl` has incorrect size");
static_assert(sizeof(physx::PxReadWriteLock) == sizeof(physx_PxReadWriteLock), "POD wrapper for `physx::PxReadWriteLock` has incorrect size");
static_assert(sizeof(physx::PxProfilerCallback) == sizeof(physx_PxProfilerCallback), "POD wrapper for `physx::PxProfilerCallback` has incorrect size");
static_assert(sizeof(physx::PxProfileScoped) == sizeof(physx_PxProfileScoped), "POD wrapper for `physx::PxProfileScoped` has incorrect size");
static_assert(sizeof(physx::PxSListEntry) == sizeof(physx_PxSListEntry), "POD wrapper for `physx::PxSListEntry` has incorrect size");
static_assert(sizeof(physx::PxSListImpl) == sizeof(physx_PxSListImpl), "POD wrapper for `physx::PxSListImpl` has incorrect size");
static_assert(sizeof(physx::PxSyncImpl) == sizeof(physx_PxSyncImpl), "POD wrapper for `physx::PxSyncImpl` has incorrect size");
static_assert(sizeof(physx::PxRunnable) == sizeof(physx_PxRunnable), "POD wrapper for `physx::PxRunnable` has incorrect size");
static_assert(sizeof(physx::PxCounterFrequencyToTensOfNanos) == sizeof(physx_PxCounterFrequencyToTensOfNanos), "POD wrapper for `physx::PxCounterFrequencyToTensOfNanos` has incorrect size");
static_assert(sizeof(physx::PxTime) == sizeof(physx_PxTime), "POD wrapper for `physx::PxTime` has incorrect size");
static_assert(sizeof(physx::PxVec2) == sizeof(physx_PxVec2), "POD wrapper for `physx::PxVec2` has incorrect size");
static_assert(sizeof(physx::PxStridedData) == sizeof(physx_PxStridedData), "POD wrapper for `physx::PxStridedData` has incorrect size");
static_assert(sizeof(physx::PxBoundedData) == sizeof(physx_PxBoundedData), "POD wrapper for `physx::PxBoundedData` has incorrect size");
static_assert(sizeof(physx::PxDebugPoint) == sizeof(physx_PxDebugPoint), "POD wrapper for `physx::PxDebugPoint` has incorrect size");
static_assert(sizeof(physx::PxDebugLine) == sizeof(physx_PxDebugLine), "POD wrapper for `physx::PxDebugLine` has incorrect size");
static_assert(sizeof(physx::PxDebugTriangle) == sizeof(physx_PxDebugTriangle), "POD wrapper for `physx::PxDebugTriangle` has incorrect size");
static_assert(sizeof(physx::PxDebugText) == sizeof(physx_PxDebugText), "POD wrapper for `physx::PxDebugText` has incorrect size");
static_assert(sizeof(physx::PxRenderBuffer) == sizeof(physx_PxRenderBuffer), "POD wrapper for `physx::PxRenderBuffer` has incorrect size");
static_assert(sizeof(physx::PxProcessPxBaseCallback) == sizeof(physx_PxProcessPxBaseCallback), "POD wrapper for `physx::PxProcessPxBaseCallback` has incorrect size");
static_assert(sizeof(physx::PxSerializationContext) == sizeof(physx_PxSerializationContext), "POD wrapper for `physx::PxSerializationContext` has incorrect size");
static_assert(sizeof(physx::PxDeserializationContext) == sizeof(physx_PxDeserializationContext), "POD wrapper for `physx::PxDeserializationContext` has incorrect size");
static_assert(sizeof(physx::PxSerializationRegistry) == sizeof(physx_PxSerializationRegistry), "POD wrapper for `physx::PxSerializationRegistry` has incorrect size");
static_assert(sizeof(physx::PxCollection) == sizeof(physx_PxCollection), "POD wrapper for `physx::PxCollection` has incorrect size");
static_assert(sizeof(physx::PxBase) == sizeof(physx_PxBase), "POD wrapper for `physx::PxBase` has incorrect size");
static_assert(sizeof(physx::PxRefCounted) == sizeof(physx_PxRefCounted), "POD wrapper for `physx::PxRefCounted` has incorrect size");
static_assert(sizeof(physx::PxTolerancesScale) == sizeof(physx_PxTolerancesScale), "POD wrapper for `physx::PxTolerancesScale` has incorrect size");
static_assert(sizeof(physx::PxStringTable) == sizeof(physx_PxStringTable), "POD wrapper for `physx::PxStringTable` has incorrect size");
static_assert(sizeof(physx::PxSerializer) == sizeof(physx_PxSerializer), "POD wrapper for `physx::PxSerializer` has incorrect size");
static_assert(sizeof(physx::PxMetaDataEntry) == sizeof(physx_PxMetaDataEntry), "POD wrapper for `physx::PxMetaDataEntry` has incorrect size");
static_assert(sizeof(physx::PxInsertionCallback) == sizeof(physx_PxInsertionCallback), "POD wrapper for `physx::PxInsertionCallback` has incorrect size");
static_assert(sizeof(physx::PxTaskManager) == sizeof(physx_PxTaskManager), "POD wrapper for `physx::PxTaskManager` has incorrect size");
static_assert(sizeof(physx::PxCpuDispatcher) == sizeof(physx_PxCpuDispatcher), "POD wrapper for `physx::PxCpuDispatcher` has incorrect size");
static_assert(sizeof(physx::PxBaseTask) == sizeof(physx_PxBaseTask), "POD wrapper for `physx::PxBaseTask` has incorrect size");
static_assert(sizeof(physx::PxTask) == sizeof(physx_PxTask), "POD wrapper for `physx::PxTask` has incorrect size");
static_assert(sizeof(physx::PxLightCpuTask) == sizeof(physx_PxLightCpuTask), "POD wrapper for `physx::PxLightCpuTask` has incorrect size");
static_assert(sizeof(physx::PxGeometry) == sizeof(physx_PxGeometry), "POD wrapper for `physx::PxGeometry` has incorrect size");
static_assert(sizeof(physx::PxBoxGeometry) == sizeof(physx_PxBoxGeometry), "POD wrapper for `physx::PxBoxGeometry` has incorrect size");
static_assert(sizeof(physx::PxBVHRaycastCallback) == sizeof(physx_PxBVHRaycastCallback), "POD wrapper for `physx::PxBVHRaycastCallback` has incorrect size");
static_assert(sizeof(physx::PxBVHOverlapCallback) == sizeof(physx_PxBVHOverlapCallback), "POD wrapper for `physx::PxBVHOverlapCallback` has incorrect size");
static_assert(sizeof(physx::PxBVHTraversalCallback) == sizeof(physx_PxBVHTraversalCallback), "POD wrapper for `physx::PxBVHTraversalCallback` has incorrect size");
static_assert(sizeof(physx::PxBVH) == sizeof(physx_PxBVH), "POD wrapper for `physx::PxBVH` has incorrect size");
static_assert(sizeof(physx::PxCapsuleGeometry) == sizeof(physx_PxCapsuleGeometry), "POD wrapper for `physx::PxCapsuleGeometry` has incorrect size");
static_assert(sizeof(physx::PxHullPolygon) == sizeof(physx_PxHullPolygon), "POD wrapper for `physx::PxHullPolygon` has incorrect size");
static_assert(sizeof(physx::PxConvexMesh) == sizeof(physx_PxConvexMesh), "POD wrapper for `physx::PxConvexMesh` has incorrect size");
static_assert(sizeof(physx::PxMeshScale) == sizeof(physx_PxMeshScale), "POD wrapper for `physx::PxMeshScale` has incorrect size");
static_assert(sizeof(physx::PxConvexMeshGeometry) == sizeof(physx_PxConvexMeshGeometry), "POD wrapper for `physx::PxConvexMeshGeometry` has incorrect size");
static_assert(sizeof(physx::PxSphereGeometry) == sizeof(physx_PxSphereGeometry), "POD wrapper for `physx::PxSphereGeometry` has incorrect size");
static_assert(sizeof(physx::PxPlaneGeometry) == sizeof(physx_PxPlaneGeometry), "POD wrapper for `physx::PxPlaneGeometry` has incorrect size");
static_assert(sizeof(physx::PxTriangleMeshGeometry) == sizeof(physx_PxTriangleMeshGeometry), "POD wrapper for `physx::PxTriangleMeshGeometry` has incorrect size");
static_assert(sizeof(physx::PxHeightFieldGeometry) == sizeof(physx_PxHeightFieldGeometry), "POD wrapper for `physx::PxHeightFieldGeometry` has incorrect size");
static_assert(sizeof(physx::PxParticleSystemGeometry) == sizeof(physx_PxParticleSystemGeometry), "POD wrapper for `physx::PxParticleSystemGeometry` has incorrect size");
static_assert(sizeof(physx::PxHairSystemGeometry) == sizeof(physx_PxHairSystemGeometry), "POD wrapper for `physx::PxHairSystemGeometry` has incorrect size");
static_assert(sizeof(physx::PxTetrahedronMeshGeometry) == sizeof(physx_PxTetrahedronMeshGeometry), "POD wrapper for `physx::PxTetrahedronMeshGeometry` has incorrect size");
static_assert(sizeof(physx::PxQueryHit) == sizeof(physx_PxQueryHit), "POD wrapper for `physx::PxQueryHit` has incorrect size");
static_assert(sizeof(physx::PxLocationHit) == sizeof(physx_PxLocationHit), "POD wrapper for `physx::PxLocationHit` has incorrect size");
static_assert(sizeof(physx::PxGeomRaycastHit) == sizeof(physx_PxGeomRaycastHit), "POD wrapper for `physx::PxGeomRaycastHit` has incorrect size");
static_assert(sizeof(physx::PxGeomOverlapHit) == sizeof(physx_PxGeomOverlapHit), "POD wrapper for `physx::PxGeomOverlapHit` has incorrect size");
static_assert(sizeof(physx::PxGeomSweepHit) == sizeof(physx_PxGeomSweepHit), "POD wrapper for `physx::PxGeomSweepHit` has incorrect size");
static_assert(sizeof(physx::PxGeomIndexPair) == sizeof(physx_PxGeomIndexPair), "POD wrapper for `physx::PxGeomIndexPair` has incorrect size");
static_assert(sizeof(physx::PxQueryThreadContext) == sizeof(physx_PxQueryThreadContext), "POD wrapper for `physx::PxQueryThreadContext` has incorrect size");
static_assert(sizeof(physx::PxCustomGeometryType) == sizeof(physx_PxCustomGeometryType), "POD wrapper for `physx::PxCustomGeometryType` has incorrect size");
static_assert(sizeof(physx::PxCustomGeometryCallbacks) == sizeof(physx_PxCustomGeometryCallbacks), "POD wrapper for `physx::PxCustomGeometryCallbacks` has incorrect size");
static_assert(sizeof(physx::PxCustomGeometry) == sizeof(physx_PxCustomGeometry), "POD wrapper for `physx::PxCustomGeometry` has incorrect size");
static_assert(sizeof(physx::PxGeometryHolder) == sizeof(physx_PxGeometryHolder), "POD wrapper for `physx::PxGeometryHolder` has incorrect size");
static_assert(sizeof(physx::PxGeometryQuery) == sizeof(physx_PxGeometryQuery), "POD wrapper for `physx::PxGeometryQuery` has incorrect size");
static_assert(sizeof(physx::PxHeightFieldSample) == sizeof(physx_PxHeightFieldSample), "POD wrapper for `physx::PxHeightFieldSample` has incorrect size");
static_assert(sizeof(physx::PxHeightField) == sizeof(physx_PxHeightField), "POD wrapper for `physx::PxHeightField` has incorrect size");
static_assert(sizeof(physx::PxHeightFieldDesc) == sizeof(physx_PxHeightFieldDesc), "POD wrapper for `physx::PxHeightFieldDesc` has incorrect size");
static_assert(sizeof(physx::PxMeshQuery) == sizeof(physx_PxMeshQuery), "POD wrapper for `physx::PxMeshQuery` has incorrect size");
static_assert(sizeof(physx::PxSimpleTriangleMesh) == sizeof(physx_PxSimpleTriangleMesh), "POD wrapper for `physx::PxSimpleTriangleMesh` has incorrect size");
static_assert(sizeof(physx::PxTriangle) == sizeof(physx_PxTriangle), "POD wrapper for `physx::PxTriangle` has incorrect size");
static_assert(sizeof(physx::PxTrianglePadded) == sizeof(physx_PxTrianglePadded), "POD wrapper for `physx::PxTrianglePadded` has incorrect size");
static_assert(sizeof(physx::PxTriangleMesh) == sizeof(physx_PxTriangleMesh), "POD wrapper for `physx::PxTriangleMesh` has incorrect size");
static_assert(sizeof(physx::PxBVH34TriangleMesh) == sizeof(physx_PxBVH34TriangleMesh), "POD wrapper for `physx::PxBVH34TriangleMesh` has incorrect size");
static_assert(sizeof(physx::PxTetrahedron) == sizeof(physx_PxTetrahedron), "POD wrapper for `physx::PxTetrahedron` has incorrect size");
static_assert(sizeof(physx::PxSoftBodyAuxData) == sizeof(physx_PxSoftBodyAuxData), "POD wrapper for `physx::PxSoftBodyAuxData` has incorrect size");
static_assert(sizeof(physx::PxTetrahedronMesh) == sizeof(physx_PxTetrahedronMesh), "POD wrapper for `physx::PxTetrahedronMesh` has incorrect size");
static_assert(sizeof(physx::PxSoftBodyMesh) == sizeof(physx_PxSoftBodyMesh), "POD wrapper for `physx::PxSoftBodyMesh` has incorrect size");
static_assert(sizeof(physx::PxCollisionMeshMappingData) == sizeof(physx_PxCollisionMeshMappingData), "POD wrapper for `physx::PxCollisionMeshMappingData` has incorrect size");
static_assert(sizeof(physx::PxSoftBodyCollisionData) == sizeof(physx_PxSoftBodyCollisionData), "POD wrapper for `physx::PxSoftBodyCollisionData` has incorrect size");
static_assert(sizeof(physx::PxTetrahedronMeshData) == sizeof(physx_PxTetrahedronMeshData), "POD wrapper for `physx::PxTetrahedronMeshData` has incorrect size");
static_assert(sizeof(physx::PxSoftBodySimulationData) == sizeof(physx_PxSoftBodySimulationData), "POD wrapper for `physx::PxSoftBodySimulationData` has incorrect size");
static_assert(sizeof(physx::PxCollisionTetrahedronMeshData) == sizeof(physx_PxCollisionTetrahedronMeshData), "POD wrapper for `physx::PxCollisionTetrahedronMeshData` has incorrect size");
static_assert(sizeof(physx::PxSimulationTetrahedronMeshData) == sizeof(physx_PxSimulationTetrahedronMeshData), "POD wrapper for `physx::PxSimulationTetrahedronMeshData` has incorrect size");
static_assert(sizeof(physx::PxActor) == sizeof(physx_PxActor), "POD wrapper for `physx::PxActor` has incorrect size");
static_assert(sizeof(physx::PxAggregate) == sizeof(physx_PxAggregate), "POD wrapper for `physx::PxAggregate` has incorrect size");
static_assert(sizeof(physx::PxSpringModifiers) == sizeof(physx_PxSpringModifiers), "POD wrapper for `physx::PxSpringModifiers` has incorrect size");
static_assert(sizeof(physx::PxRestitutionModifiers) == sizeof(physx_PxRestitutionModifiers), "POD wrapper for `physx::PxRestitutionModifiers` has incorrect size");
static_assert(sizeof(physx::Px1DConstraintMods) == sizeof(physx_Px1DConstraintMods), "POD wrapper for `physx::Px1DConstraintMods` has incorrect size");
static_assert(sizeof(physx::Px1DConstraint) == sizeof(physx_Px1DConstraint), "POD wrapper for `physx::Px1DConstraint` has incorrect size");
static_assert(sizeof(physx::PxConstraintInvMassScale) == sizeof(physx_PxConstraintInvMassScale), "POD wrapper for `physx::PxConstraintInvMassScale` has incorrect size");
static_assert(sizeof(physx::PxConstraintVisualizer) == sizeof(physx_PxConstraintVisualizer), "POD wrapper for `physx::PxConstraintVisualizer` has incorrect size");
static_assert(sizeof(physx::PxConstraintConnector) == sizeof(physx_PxConstraintConnector), "POD wrapper for `physx::PxConstraintConnector` has incorrect size");
static_assert(sizeof(physx::PxContactPoint) == sizeof(physx_PxContactPoint), "POD wrapper for `physx::PxContactPoint` has incorrect size");
static_assert(sizeof(physx::PxSolverBody) == sizeof(physx_PxSolverBody), "POD wrapper for `physx::PxSolverBody` has incorrect size");
static_assert(sizeof(physx::PxSolverBodyData) == sizeof(physx_PxSolverBodyData), "POD wrapper for `physx::PxSolverBodyData` has incorrect size");
static_assert(sizeof(physx::PxConstraintBatchHeader) == sizeof(physx_PxConstraintBatchHeader), "POD wrapper for `physx::PxConstraintBatchHeader` has incorrect size");
static_assert(sizeof(physx::PxSolverConstraintDesc) == sizeof(physx_PxSolverConstraintDesc), "POD wrapper for `physx::PxSolverConstraintDesc` has incorrect size");
static_assert(sizeof(physx::PxSolverConstraintPrepDescBase) == sizeof(physx_PxSolverConstraintPrepDescBase), "POD wrapper for `physx::PxSolverConstraintPrepDescBase` has incorrect size");
static_assert(sizeof(physx::PxSolverConstraintPrepDesc) == sizeof(physx_PxSolverConstraintPrepDesc), "POD wrapper for `physx::PxSolverConstraintPrepDesc` has incorrect size");
static_assert(sizeof(physx::PxSolverContactDesc) == sizeof(physx_PxSolverContactDesc), "POD wrapper for `physx::PxSolverContactDesc` has incorrect size");
static_assert(sizeof(physx::PxConstraintAllocator) == sizeof(physx_PxConstraintAllocator), "POD wrapper for `physx::PxConstraintAllocator` has incorrect size");
static_assert(sizeof(physx::PxArticulationLimit) == sizeof(physx_PxArticulationLimit), "POD wrapper for `physx::PxArticulationLimit` has incorrect size");
static_assert(sizeof(physx::PxArticulationDrive) == sizeof(physx_PxArticulationDrive), "POD wrapper for `physx::PxArticulationDrive` has incorrect size");
static_assert(sizeof(physx::PxTGSSolverBodyVel) == sizeof(physx_PxTGSSolverBodyVel), "POD wrapper for `physx::PxTGSSolverBodyVel` has incorrect size");
static_assert(sizeof(physx::PxTGSSolverBodyTxInertia) == sizeof(physx_PxTGSSolverBodyTxInertia), "POD wrapper for `physx::PxTGSSolverBodyTxInertia` has incorrect size");
static_assert(sizeof(physx::PxTGSSolverBodyData) == sizeof(physx_PxTGSSolverBodyData), "POD wrapper for `physx::PxTGSSolverBodyData` has incorrect size");
static_assert(sizeof(physx::PxTGSSolverConstraintPrepDescBase) == sizeof(physx_PxTGSSolverConstraintPrepDescBase), "POD wrapper for `physx::PxTGSSolverConstraintPrepDescBase` has incorrect size");
static_assert(sizeof(physx::PxTGSSolverConstraintPrepDesc) == sizeof(physx_PxTGSSolverConstraintPrepDesc), "POD wrapper for `physx::PxTGSSolverConstraintPrepDesc` has incorrect size");
static_assert(sizeof(physx::PxTGSSolverContactDesc) == sizeof(physx_PxTGSSolverContactDesc), "POD wrapper for `physx::PxTGSSolverContactDesc` has incorrect size");
static_assert(sizeof(physx::PxArticulationTendonLimit) == sizeof(physx_PxArticulationTendonLimit), "POD wrapper for `physx::PxArticulationTendonLimit` has incorrect size");
static_assert(sizeof(physx::PxArticulationAttachment) == sizeof(physx_PxArticulationAttachment), "POD wrapper for `physx::PxArticulationAttachment` has incorrect size");
static_assert(sizeof(physx::PxArticulationTendonJoint) == sizeof(physx_PxArticulationTendonJoint), "POD wrapper for `physx::PxArticulationTendonJoint` has incorrect size");
static_assert(sizeof(physx::PxArticulationTendon) == sizeof(physx_PxArticulationTendon), "POD wrapper for `physx::PxArticulationTendon` has incorrect size");
static_assert(sizeof(physx::PxArticulationSpatialTendon) == sizeof(physx_PxArticulationSpatialTendon), "POD wrapper for `physx::PxArticulationSpatialTendon` has incorrect size");
static_assert(sizeof(physx::PxArticulationFixedTendon) == sizeof(physx_PxArticulationFixedTendon), "POD wrapper for `physx::PxArticulationFixedTendon` has incorrect size");
static_assert(sizeof(physx::PxSpatialForce) == sizeof(physx_PxSpatialForce), "POD wrapper for `physx::PxSpatialForce` has incorrect size");
static_assert(sizeof(physx::PxSpatialVelocity) == sizeof(physx_PxSpatialVelocity), "POD wrapper for `physx::PxSpatialVelocity` has incorrect size");
static_assert(sizeof(physx::PxArticulationRootLinkData) == sizeof(physx_PxArticulationRootLinkData), "POD wrapper for `physx::PxArticulationRootLinkData` has incorrect size");
static_assert(sizeof(physx::PxArticulationCache) == sizeof(physx_PxArticulationCache), "POD wrapper for `physx::PxArticulationCache` has incorrect size");
static_assert(sizeof(physx::PxArticulationSensor) == sizeof(physx_PxArticulationSensor), "POD wrapper for `physx::PxArticulationSensor` has incorrect size");
static_assert(sizeof(physx::PxArticulationReducedCoordinate) == sizeof(physx_PxArticulationReducedCoordinate), "POD wrapper for `physx::PxArticulationReducedCoordinate` has incorrect size");
static_assert(sizeof(physx::PxArticulationJointReducedCoordinate) == sizeof(physx_PxArticulationJointReducedCoordinate), "POD wrapper for `physx::PxArticulationJointReducedCoordinate` has incorrect size");
static_assert(sizeof(physx::PxShape) == sizeof(physx_PxShape), "POD wrapper for `physx::PxShape` has incorrect size");
static_assert(sizeof(physx::PxRigidActor) == sizeof(physx_PxRigidActor), "POD wrapper for `physx::PxRigidActor` has incorrect size");
static_assert(sizeof(physx::PxNodeIndex) == sizeof(physx_PxNodeIndex), "POD wrapper for `physx::PxNodeIndex` has incorrect size");
static_assert(sizeof(physx::PxRigidBody) == sizeof(physx_PxRigidBody), "POD wrapper for `physx::PxRigidBody` has incorrect size");
static_assert(sizeof(physx::PxArticulationLink) == sizeof(physx_PxArticulationLink), "POD wrapper for `physx::PxArticulationLink` has incorrect size");
static_assert(sizeof(physx::PxConeLimitedConstraint) == sizeof(physx_PxConeLimitedConstraint), "POD wrapper for `physx::PxConeLimitedConstraint` has incorrect size");
static_assert(sizeof(physx::PxConeLimitParams) == sizeof(physx_PxConeLimitParams), "POD wrapper for `physx::PxConeLimitParams` has incorrect size");
static_assert(sizeof(physx::PxConstraintShaderTable) == sizeof(physx_PxConstraintShaderTable), "POD wrapper for `physx::PxConstraintShaderTable` has incorrect size");
static_assert(sizeof(physx::PxConstraint) == sizeof(physx_PxConstraint), "POD wrapper for `physx::PxConstraint` has incorrect size");
static_assert(sizeof(physx::PxMassModificationProps) == sizeof(physx_PxMassModificationProps), "POD wrapper for `physx::PxMassModificationProps` has incorrect size");
static_assert(sizeof(physx::PxContactPatch) == sizeof(physx_PxContactPatch), "POD wrapper for `physx::PxContactPatch` has incorrect size");
static_assert(sizeof(physx::PxContact) == sizeof(physx_PxContact), "POD wrapper for `physx::PxContact` has incorrect size");
static_assert(sizeof(physx::PxExtendedContact) == sizeof(physx_PxExtendedContact), "POD wrapper for `physx::PxExtendedContact` has incorrect size");
static_assert(sizeof(physx::PxModifiableContact) == sizeof(physx_PxModifiableContact), "POD wrapper for `physx::PxModifiableContact` has incorrect size");
static_assert(sizeof(physx::PxContactStreamIterator) == sizeof(physx_PxContactStreamIterator), "POD wrapper for `physx::PxContactStreamIterator` has incorrect size");
static_assert(sizeof(physx::PxGpuContactPair) == sizeof(physx_PxGpuContactPair), "POD wrapper for `physx::PxGpuContactPair` has incorrect size");
static_assert(sizeof(physx::PxContactSet) == sizeof(physx_PxContactSet), "POD wrapper for `physx::PxContactSet` has incorrect size");
static_assert(sizeof(physx::PxContactModifyPair) == sizeof(physx_PxContactModifyPair), "POD wrapper for `physx::PxContactModifyPair` has incorrect size");
static_assert(sizeof(physx::PxContactModifyCallback) == sizeof(physx_PxContactModifyCallback), "POD wrapper for `physx::PxContactModifyCallback` has incorrect size");
static_assert(sizeof(physx::PxCCDContactModifyCallback) == sizeof(physx_PxCCDContactModifyCallback), "POD wrapper for `physx::PxCCDContactModifyCallback` has incorrect size");
static_assert(sizeof(physx::PxDeletionListener) == sizeof(physx_PxDeletionListener), "POD wrapper for `physx::PxDeletionListener` has incorrect size");
static_assert(sizeof(physx::PxBaseMaterial) == sizeof(physx_PxBaseMaterial), "POD wrapper for `physx::PxBaseMaterial` has incorrect size");
static_assert(sizeof(physx::PxFEMMaterial) == sizeof(physx_PxFEMMaterial), "POD wrapper for `physx::PxFEMMaterial` has incorrect size");
static_assert(sizeof(physx::PxFilterData) == sizeof(physx_PxFilterData), "POD wrapper for `physx::PxFilterData` has incorrect size");
static_assert(sizeof(physx::PxSimulationFilterCallback) == sizeof(physx_PxSimulationFilterCallback), "POD wrapper for `physx::PxSimulationFilterCallback` has incorrect size");
static_assert(sizeof(physx::PxParticleRigidFilterPair) == sizeof(physx_PxParticleRigidFilterPair), "POD wrapper for `physx::PxParticleRigidFilterPair` has incorrect size");
static_assert(sizeof(physx::PxLockedData) == sizeof(physx_PxLockedData), "POD wrapper for `physx::PxLockedData` has incorrect size");
static_assert(sizeof(physx::PxMaterial) == sizeof(physx_PxMaterial), "POD wrapper for `physx::PxMaterial` has incorrect size");
static_assert(sizeof(physx::PxGpuParticleBufferIndexPair) == sizeof(physx_PxGpuParticleBufferIndexPair), "POD wrapper for `physx::PxGpuParticleBufferIndexPair` has incorrect size");
static_assert(sizeof(physx::PxParticleVolume) == sizeof(physx_PxParticleVolume), "POD wrapper for `physx::PxParticleVolume` has incorrect size");
static_assert(sizeof(physx::PxDiffuseParticleParams) == sizeof(physx_PxDiffuseParticleParams), "POD wrapper for `physx::PxDiffuseParticleParams` has incorrect size");
static_assert(sizeof(physx::PxParticleSpring) == sizeof(physx_PxParticleSpring), "POD wrapper for `physx::PxParticleSpring` has incorrect size");
static_assert(sizeof(physx::PxParticleMaterial) == sizeof(physx_PxParticleMaterial), "POD wrapper for `physx::PxParticleMaterial` has incorrect size");
static_assert(sizeof(physx::PxPhysics) == sizeof(physx_PxPhysics), "POD wrapper for `physx::PxPhysics` has incorrect size");
static_assert(sizeof(physx::PxActorShape) == sizeof(physx_PxActorShape), "POD wrapper for `physx::PxActorShape` has incorrect size");
static_assert(sizeof(physx::PxRaycastHit) == sizeof(physx_PxRaycastHit), "POD wrapper for `physx::PxRaycastHit` has incorrect size");
static_assert(sizeof(physx::PxOverlapHit) == sizeof(physx_PxOverlapHit), "POD wrapper for `physx::PxOverlapHit` has incorrect size");
static_assert(sizeof(physx::PxSweepHit) == sizeof(physx_PxSweepHit), "POD wrapper for `physx::PxSweepHit` has incorrect size");
static_assert(sizeof(physx::PxRaycastCallback) == sizeof(physx_PxRaycastCallback), "POD wrapper for `physx::PxRaycastCallback` has incorrect size");
static_assert(sizeof(physx::PxOverlapCallback) == sizeof(physx_PxOverlapCallback), "POD wrapper for `physx::PxOverlapCallback` has incorrect size");
static_assert(sizeof(physx::PxSweepCallback) == sizeof(physx_PxSweepCallback), "POD wrapper for `physx::PxSweepCallback` has incorrect size");
static_assert(sizeof(physx::PxRaycastBuffer) == sizeof(physx_PxRaycastBuffer), "POD wrapper for `physx::PxRaycastBuffer` has incorrect size");
static_assert(sizeof(physx::PxOverlapBuffer) == sizeof(physx_PxOverlapBuffer), "POD wrapper for `physx::PxOverlapBuffer` has incorrect size");
static_assert(sizeof(physx::PxSweepBuffer) == sizeof(physx_PxSweepBuffer), "POD wrapper for `physx::PxSweepBuffer` has incorrect size");
static_assert(sizeof(physx::PxQueryCache) == sizeof(physx_PxQueryCache), "POD wrapper for `physx::PxQueryCache` has incorrect size");
static_assert(sizeof(physx::PxQueryFilterData) == sizeof(physx_PxQueryFilterData), "POD wrapper for `physx::PxQueryFilterData` has incorrect size");
static_assert(sizeof(physx::PxQueryFilterCallback) == sizeof(physx_PxQueryFilterCallback), "POD wrapper for `physx::PxQueryFilterCallback` has incorrect size");
static_assert(sizeof(physx::PxRigidDynamic) == sizeof(physx_PxRigidDynamic), "POD wrapper for `physx::PxRigidDynamic` has incorrect size");
static_assert(sizeof(physx::PxRigidStatic) == sizeof(physx_PxRigidStatic), "POD wrapper for `physx::PxRigidStatic` has incorrect size");
static_assert(sizeof(physx::PxSceneQueryDesc) == sizeof(physx_PxSceneQueryDesc), "POD wrapper for `physx::PxSceneQueryDesc` has incorrect size");
static_assert(sizeof(physx::PxSceneQuerySystemBase) == sizeof(physx_PxSceneQuerySystemBase), "POD wrapper for `physx::PxSceneQuerySystemBase` has incorrect size");
static_assert(sizeof(physx::PxSceneSQSystem) == sizeof(physx_PxSceneSQSystem), "POD wrapper for `physx::PxSceneSQSystem` has incorrect size");
static_assert(sizeof(physx::PxSceneQuerySystem) == sizeof(physx_PxSceneQuerySystem), "POD wrapper for `physx::PxSceneQuerySystem` has incorrect size");
static_assert(sizeof(physx::PxBroadPhaseRegion) == sizeof(physx_PxBroadPhaseRegion), "POD wrapper for `physx::PxBroadPhaseRegion` has incorrect size");
static_assert(sizeof(physx::PxBroadPhaseRegionInfo) == sizeof(physx_PxBroadPhaseRegionInfo), "POD wrapper for `physx::PxBroadPhaseRegionInfo` has incorrect size");
static_assert(sizeof(physx::PxBroadPhaseCaps) == sizeof(physx_PxBroadPhaseCaps), "POD wrapper for `physx::PxBroadPhaseCaps` has incorrect size");
static_assert(sizeof(physx::PxBroadPhaseDesc) == sizeof(physx_PxBroadPhaseDesc), "POD wrapper for `physx::PxBroadPhaseDesc` has incorrect size");
static_assert(sizeof(physx::PxBroadPhaseUpdateData) == sizeof(physx_PxBroadPhaseUpdateData), "POD wrapper for `physx::PxBroadPhaseUpdateData` has incorrect size");
static_assert(sizeof(physx::PxBroadPhasePair) == sizeof(physx_PxBroadPhasePair), "POD wrapper for `physx::PxBroadPhasePair` has incorrect size");
static_assert(sizeof(physx::PxBroadPhaseResults) == sizeof(physx_PxBroadPhaseResults), "POD wrapper for `physx::PxBroadPhaseResults` has incorrect size");
static_assert(sizeof(physx::PxBroadPhaseRegions) == sizeof(physx_PxBroadPhaseRegions), "POD wrapper for `physx::PxBroadPhaseRegions` has incorrect size");
static_assert(sizeof(physx::PxBroadPhase) == sizeof(physx_PxBroadPhase), "POD wrapper for `physx::PxBroadPhase` has incorrect size");
static_assert(sizeof(physx::PxAABBManager) == sizeof(physx_PxAABBManager), "POD wrapper for `physx::PxAABBManager` has incorrect size");
static_assert(sizeof(physx::PxSceneLimits) == sizeof(physx_PxSceneLimits), "POD wrapper for `physx::PxSceneLimits` has incorrect size");
static_assert(sizeof(physx::PxgDynamicsMemoryConfig) == sizeof(physx_PxgDynamicsMemoryConfig), "POD wrapper for `physx::PxgDynamicsMemoryConfig` has incorrect size");
static_assert(sizeof(physx::PxSceneDesc) == sizeof(physx_PxSceneDesc), "POD wrapper for `physx::PxSceneDesc` has incorrect size");
static_assert(sizeof(physx::PxSimulationStatistics) == sizeof(physx_PxSimulationStatistics), "POD wrapper for `physx::PxSimulationStatistics` has incorrect size");
static_assert(sizeof(physx::PxGpuBodyData) == sizeof(physx_PxGpuBodyData), "POD wrapper for `physx::PxGpuBodyData` has incorrect size");
static_assert(sizeof(physx::PxGpuActorPair) == sizeof(physx_PxGpuActorPair), "POD wrapper for `physx::PxGpuActorPair` has incorrect size");
static_assert(sizeof(physx::PxIndexDataPair) == sizeof(physx_PxIndexDataPair), "POD wrapper for `physx::PxIndexDataPair` has incorrect size");
static_assert(sizeof(physx::PxPvdSceneClient) == sizeof(physx_PxPvdSceneClient), "POD wrapper for `physx::PxPvdSceneClient` has incorrect size");
static_assert(sizeof(physx::PxDominanceGroupPair) == sizeof(physx_PxDominanceGroupPair), "POD wrapper for `physx::PxDominanceGroupPair` has incorrect size");
static_assert(sizeof(physx::PxBroadPhaseCallback) == sizeof(physx_PxBroadPhaseCallback), "POD wrapper for `physx::PxBroadPhaseCallback` has incorrect size");
static_assert(sizeof(physx::PxScene) == sizeof(physx_PxScene), "POD wrapper for `physx::PxScene` has incorrect size");
static_assert(sizeof(physx::PxSceneReadLock) == sizeof(physx_PxSceneReadLock), "POD wrapper for `physx::PxSceneReadLock` has incorrect size");
static_assert(sizeof(physx::PxSceneWriteLock) == sizeof(physx_PxSceneWriteLock), "POD wrapper for `physx::PxSceneWriteLock` has incorrect size");
static_assert(sizeof(physx::PxContactPairExtraDataItem) == sizeof(physx_PxContactPairExtraDataItem), "POD wrapper for `physx::PxContactPairExtraDataItem` has incorrect size");
static_assert(sizeof(physx::PxContactPairVelocity) == sizeof(physx_PxContactPairVelocity), "POD wrapper for `physx::PxContactPairVelocity` has incorrect size");
static_assert(sizeof(physx::PxContactPairPose) == sizeof(physx_PxContactPairPose), "POD wrapper for `physx::PxContactPairPose` has incorrect size");
static_assert(sizeof(physx::PxContactPairIndex) == sizeof(physx_PxContactPairIndex), "POD wrapper for `physx::PxContactPairIndex` has incorrect size");
static_assert(sizeof(physx::PxContactPairExtraDataIterator) == sizeof(physx_PxContactPairExtraDataIterator), "POD wrapper for `physx::PxContactPairExtraDataIterator` has incorrect size");
static_assert(sizeof(physx::PxContactPairHeader) == sizeof(physx_PxContactPairHeader), "POD wrapper for `physx::PxContactPairHeader` has incorrect size");
static_assert(sizeof(physx::PxContactPairPoint) == sizeof(physx_PxContactPairPoint), "POD wrapper for `physx::PxContactPairPoint` has incorrect size");
static_assert(sizeof(physx::PxContactPair) == sizeof(physx_PxContactPair), "POD wrapper for `physx::PxContactPair` has incorrect size");
static_assert(sizeof(physx::PxTriggerPair) == sizeof(physx_PxTriggerPair), "POD wrapper for `physx::PxTriggerPair` has incorrect size");
static_assert(sizeof(physx::PxConstraintInfo) == sizeof(physx_PxConstraintInfo), "POD wrapper for `physx::PxConstraintInfo` has incorrect size");
static_assert(sizeof(physx::PxSimulationEventCallback) == sizeof(physx_PxSimulationEventCallback), "POD wrapper for `physx::PxSimulationEventCallback` has incorrect size");
static_assert(sizeof(physx::PxFEMParameters) == sizeof(physx_PxFEMParameters), "POD wrapper for `physx::PxFEMParameters` has incorrect size");
static_assert(sizeof(physx::PxPruningStructure) == sizeof(physx_PxPruningStructure), "POD wrapper for `physx::PxPruningStructure` has incorrect size");
static_assert(sizeof(physx::PxExtendedVec3) == sizeof(physx_PxExtendedVec3), "POD wrapper for `physx::PxExtendedVec3` has incorrect size");
static_assert(sizeof(physx::PxObstacle) == sizeof(physx_PxObstacle), "POD wrapper for `physx::PxObstacle` has incorrect size");
static_assert(sizeof(physx::PxBoxObstacle) == sizeof(physx_PxBoxObstacle), "POD wrapper for `physx::PxBoxObstacle` has incorrect size");
static_assert(sizeof(physx::PxCapsuleObstacle) == sizeof(physx_PxCapsuleObstacle), "POD wrapper for `physx::PxCapsuleObstacle` has incorrect size");
static_assert(sizeof(physx::PxObstacleContext) == sizeof(physx_PxObstacleContext), "POD wrapper for `physx::PxObstacleContext` has incorrect size");
static_assert(sizeof(physx::PxControllerState) == sizeof(physx_PxControllerState), "POD wrapper for `physx::PxControllerState` has incorrect size");
static_assert(sizeof(physx::PxControllerStats) == sizeof(physx_PxControllerStats), "POD wrapper for `physx::PxControllerStats` has incorrect size");
static_assert(sizeof(physx::PxControllerHit) == sizeof(physx_PxControllerHit), "POD wrapper for `physx::PxControllerHit` has incorrect size");
static_assert(sizeof(physx::PxControllerShapeHit) == sizeof(physx_PxControllerShapeHit), "POD wrapper for `physx::PxControllerShapeHit` has incorrect size");
static_assert(sizeof(physx::PxControllersHit) == sizeof(physx_PxControllersHit), "POD wrapper for `physx::PxControllersHit` has incorrect size");
static_assert(sizeof(physx::PxControllerObstacleHit) == sizeof(physx_PxControllerObstacleHit), "POD wrapper for `physx::PxControllerObstacleHit` has incorrect size");
static_assert(sizeof(physx::PxUserControllerHitReport) == sizeof(physx_PxUserControllerHitReport), "POD wrapper for `physx::PxUserControllerHitReport` has incorrect size");
static_assert(sizeof(physx::PxControllerFilterCallback) == sizeof(physx_PxControllerFilterCallback), "POD wrapper for `physx::PxControllerFilterCallback` has incorrect size");
static_assert(sizeof(physx::PxControllerFilters) == sizeof(physx_PxControllerFilters), "POD wrapper for `physx::PxControllerFilters` has incorrect size");
static_assert(sizeof(physx::PxControllerDesc) == sizeof(physx_PxControllerDesc), "POD wrapper for `physx::PxControllerDesc` has incorrect size");
static_assert(sizeof(physx::PxController) == sizeof(physx_PxController), "POD wrapper for `physx::PxController` has incorrect size");
static_assert(sizeof(physx::PxBoxControllerDesc) == sizeof(physx_PxBoxControllerDesc), "POD wrapper for `physx::PxBoxControllerDesc` has incorrect size");
static_assert(sizeof(physx::PxBoxController) == sizeof(physx_PxBoxController), "POD wrapper for `physx::PxBoxController` has incorrect size");
static_assert(sizeof(physx::PxCapsuleControllerDesc) == sizeof(physx_PxCapsuleControllerDesc), "POD wrapper for `physx::PxCapsuleControllerDesc` has incorrect size");
static_assert(sizeof(physx::PxCapsuleController) == sizeof(physx_PxCapsuleController), "POD wrapper for `physx::PxCapsuleController` has incorrect size");
static_assert(sizeof(physx::PxControllerBehaviorCallback) == sizeof(physx_PxControllerBehaviorCallback), "POD wrapper for `physx::PxControllerBehaviorCallback` has incorrect size");
static_assert(sizeof(physx::PxControllerManager) == sizeof(physx_PxControllerManager), "POD wrapper for `physx::PxControllerManager` has incorrect size");
static_assert(sizeof(physx::PxDim3) == sizeof(physx_PxDim3), "POD wrapper for `physx::PxDim3` has incorrect size");
static_assert(sizeof(physx::PxSDFDesc) == sizeof(physx_PxSDFDesc), "POD wrapper for `physx::PxSDFDesc` has incorrect size");
static_assert(sizeof(physx::PxConvexMeshDesc) == sizeof(physx_PxConvexMeshDesc), "POD wrapper for `physx::PxConvexMeshDesc` has incorrect size");
static_assert(sizeof(physx::PxTriangleMeshDesc) == sizeof(physx_PxTriangleMeshDesc), "POD wrapper for `physx::PxTriangleMeshDesc` has incorrect size");
static_assert(sizeof(physx::PxTetrahedronMeshDesc) == sizeof(physx_PxTetrahedronMeshDesc), "POD wrapper for `physx::PxTetrahedronMeshDesc` has incorrect size");
static_assert(sizeof(physx::PxSoftBodySimulationDataDesc) == sizeof(physx_PxSoftBodySimulationDataDesc), "POD wrapper for `physx::PxSoftBodySimulationDataDesc` has incorrect size");
static_assert(sizeof(physx::PxBVH34MidphaseDesc) == sizeof(physx_PxBVH34MidphaseDesc), "POD wrapper for `physx::PxBVH34MidphaseDesc` has incorrect size");
static_assert(sizeof(physx::PxMidphaseDesc) == sizeof(physx_PxMidphaseDesc), "POD wrapper for `physx::PxMidphaseDesc` has incorrect size");
static_assert(sizeof(physx::PxBVHDesc) == sizeof(physx_PxBVHDesc), "POD wrapper for `physx::PxBVHDesc` has incorrect size");
static_assert(sizeof(physx::PxCookingParams) == sizeof(physx_PxCookingParams), "POD wrapper for `physx::PxCookingParams` has incorrect size");
static_assert(sizeof(physx::PxDefaultMemoryOutputStream) == sizeof(physx_PxDefaultMemoryOutputStream), "POD wrapper for `physx::PxDefaultMemoryOutputStream` has incorrect size");
static_assert(sizeof(physx::PxDefaultMemoryInputData) == sizeof(physx_PxDefaultMemoryInputData), "POD wrapper for `physx::PxDefaultMemoryInputData` has incorrect size");
static_assert(sizeof(physx::PxDefaultFileOutputStream) == sizeof(physx_PxDefaultFileOutputStream), "POD wrapper for `physx::PxDefaultFileOutputStream` has incorrect size");
static_assert(sizeof(physx::PxDefaultFileInputData) == sizeof(physx_PxDefaultFileInputData), "POD wrapper for `physx::PxDefaultFileInputData` has incorrect size");
static_assert(sizeof(physx::PxDefaultAllocator) == sizeof(physx_PxDefaultAllocator), "POD wrapper for `physx::PxDefaultAllocator` has incorrect size");
static_assert(sizeof(physx::PxJoint) == sizeof(physx_PxJoint), "POD wrapper for `physx::PxJoint` has incorrect size");
static_assert(sizeof(physx::PxSpring) == sizeof(physx_PxSpring), "POD wrapper for `physx::PxSpring` has incorrect size");
static_assert(sizeof(physx::PxDistanceJoint) == sizeof(physx_PxDistanceJoint), "POD wrapper for `physx::PxDistanceJoint` has incorrect size");
static_assert(sizeof(physx::PxJacobianRow) == sizeof(physx_PxJacobianRow), "POD wrapper for `physx::PxJacobianRow` has incorrect size");
static_assert(sizeof(physx::PxContactJoint) == sizeof(physx_PxContactJoint), "POD wrapper for `physx::PxContactJoint` has incorrect size");
static_assert(sizeof(physx::PxFixedJoint) == sizeof(physx_PxFixedJoint), "POD wrapper for `physx::PxFixedJoint` has incorrect size");
static_assert(sizeof(physx::PxJointLimitParameters) == sizeof(physx_PxJointLimitParameters), "POD wrapper for `physx::PxJointLimitParameters` has incorrect size");
static_assert(sizeof(physx::PxJointLinearLimit) == sizeof(physx_PxJointLinearLimit), "POD wrapper for `physx::PxJointLinearLimit` has incorrect size");
static_assert(sizeof(physx::PxJointLinearLimitPair) == sizeof(physx_PxJointLinearLimitPair), "POD wrapper for `physx::PxJointLinearLimitPair` has incorrect size");
static_assert(sizeof(physx::PxJointAngularLimitPair) == sizeof(physx_PxJointAngularLimitPair), "POD wrapper for `physx::PxJointAngularLimitPair` has incorrect size");
static_assert(sizeof(physx::PxJointLimitCone) == sizeof(physx_PxJointLimitCone), "POD wrapper for `physx::PxJointLimitCone` has incorrect size");
static_assert(sizeof(physx::PxJointLimitPyramid) == sizeof(physx_PxJointLimitPyramid), "POD wrapper for `physx::PxJointLimitPyramid` has incorrect size");
static_assert(sizeof(physx::PxPrismaticJoint) == sizeof(physx_PxPrismaticJoint), "POD wrapper for `physx::PxPrismaticJoint` has incorrect size");
static_assert(sizeof(physx::PxRevoluteJoint) == sizeof(physx_PxRevoluteJoint), "POD wrapper for `physx::PxRevoluteJoint` has incorrect size");
static_assert(sizeof(physx::PxSphericalJoint) == sizeof(physx_PxSphericalJoint), "POD wrapper for `physx::PxSphericalJoint` has incorrect size");
static_assert(sizeof(physx::PxD6JointDrive) == sizeof(physx_PxD6JointDrive), "POD wrapper for `physx::PxD6JointDrive` has incorrect size");
static_assert(sizeof(physx::PxD6Joint) == sizeof(physx_PxD6Joint), "POD wrapper for `physx::PxD6Joint` has incorrect size");
static_assert(sizeof(physx::PxGearJoint) == sizeof(physx_PxGearJoint), "POD wrapper for `physx::PxGearJoint` has incorrect size");
static_assert(sizeof(physx::PxRackAndPinionJoint) == sizeof(physx_PxRackAndPinionJoint), "POD wrapper for `physx::PxRackAndPinionJoint` has incorrect size");
static_assert(sizeof(physx::PxGroupsMask) == sizeof(physx_PxGroupsMask), "POD wrapper for `physx::PxGroupsMask` has incorrect size");
static_assert(sizeof(physx::PxDefaultErrorCallback) == sizeof(physx_PxDefaultErrorCallback), "POD wrapper for `physx::PxDefaultErrorCallback` has incorrect size");
static_assert(sizeof(physx::PxRigidActorExt) == sizeof(physx_PxRigidActorExt), "POD wrapper for `physx::PxRigidActorExt` has incorrect size");
static_assert(sizeof(physx::PxMassProperties) == sizeof(physx_PxMassProperties), "POD wrapper for `physx::PxMassProperties` has incorrect size");
static_assert(sizeof(physx::PxRigidBodyExt) == sizeof(physx_PxRigidBodyExt), "POD wrapper for `physx::PxRigidBodyExt` has incorrect size");
static_assert(sizeof(physx::PxShapeExt) == sizeof(physx_PxShapeExt), "POD wrapper for `physx::PxShapeExt` has incorrect size");
static_assert(sizeof(physx::PxMeshOverlapUtil) == sizeof(physx_PxMeshOverlapUtil), "POD wrapper for `physx::PxMeshOverlapUtil` has incorrect size");
static_assert(sizeof(physx::PxXmlMiscParameter) == sizeof(physx_PxXmlMiscParameter), "POD wrapper for `physx::PxXmlMiscParameter` has incorrect size");
static_assert(sizeof(physx::PxSerialization) == sizeof(physx_PxSerialization), "POD wrapper for `physx::PxSerialization` has incorrect size");
static_assert(sizeof(physx::PxDefaultCpuDispatcher) == sizeof(physx_PxDefaultCpuDispatcher), "POD wrapper for `physx::PxDefaultCpuDispatcher` has incorrect size");
static_assert(sizeof(physx::PxStringTableExt) == sizeof(physx_PxStringTableExt), "POD wrapper for `physx::PxStringTableExt` has incorrect size");
static_assert(sizeof(physx::PxBroadPhaseExt) == sizeof(physx_PxBroadPhaseExt), "POD wrapper for `physx::PxBroadPhaseExt` has incorrect size");
static_assert(sizeof(physx::PxSceneQueryExt) == sizeof(physx_PxSceneQueryExt), "POD wrapper for `physx::PxSceneQueryExt` has incorrect size");
static_assert(sizeof(physx::PxBatchQueryExt) == sizeof(physx_PxBatchQueryExt), "POD wrapper for `physx::PxBatchQueryExt` has incorrect size");
static_assert(sizeof(physx::PxCustomSceneQuerySystem) == sizeof(physx_PxCustomSceneQuerySystem), "POD wrapper for `physx::PxCustomSceneQuerySystem` has incorrect size");
static_assert(sizeof(physx::PxCustomSceneQuerySystemAdapter) == sizeof(physx_PxCustomSceneQuerySystemAdapter), "POD wrapper for `physx::PxCustomSceneQuerySystemAdapter` has incorrect size");
static_assert(sizeof(physx::PxSamplingExt) == sizeof(physx_PxSamplingExt), "POD wrapper for `physx::PxSamplingExt` has incorrect size");
static_assert(sizeof(physx::PxPoissonSampler) == sizeof(physx_PxPoissonSampler), "POD wrapper for `physx::PxPoissonSampler` has incorrect size");
static_assert(sizeof(physx::PxTriangleMeshPoissonSampler) == sizeof(physx_PxTriangleMeshPoissonSampler), "POD wrapper for `physx::PxTriangleMeshPoissonSampler` has incorrect size");
static_assert(sizeof(physx::PxTetrahedronMeshExt) == sizeof(physx_PxTetrahedronMeshExt), "POD wrapper for `physx::PxTetrahedronMeshExt` has incorrect size");
static_assert(sizeof(physx::PxRepXObject) == sizeof(physx_PxRepXObject), "POD wrapper for `physx::PxRepXObject` has incorrect size");
static_assert(sizeof(physx::PxRepXInstantiationArgs) == sizeof(physx_PxRepXInstantiationArgs), "POD wrapper for `physx::PxRepXInstantiationArgs` has incorrect size");
static_assert(sizeof(physx::PxRepXSerializer) == sizeof(physx_PxRepXSerializer), "POD wrapper for `physx::PxRepXSerializer` has incorrect size");
static_assert(sizeof(physx::PxPvd) == sizeof(physx_PxPvd), "POD wrapper for `physx::PxPvd` has incorrect size");
static_assert(sizeof(physx::PxPvdTransport) == sizeof(physx_PxPvdTransport), "POD wrapper for `physx::PxPvdTransport` has incorrect size");

extern "C" {
    void PxAllocatorCallback_delete(physx_PxAllocatorCallback* self__pod) {
        physx::PxAllocatorCallback* self_ = reinterpret_cast<physx::PxAllocatorCallback*>(self__pod);
        delete self_;
    }

    void* PxAllocatorCallback_allocate(physx_PxAllocatorCallback* self__pod, size_t size_pod, char const* typeName, char const* filename, int32_t line) {
        physx::PxAllocatorCallback* self_ = reinterpret_cast<physx::PxAllocatorCallback*>(self__pod);
        size_t size;
        memcpy(&size, &size_pod, sizeof(size));
        void* return_val = self_->allocate(size, typeName, filename, line);
        return return_val;
    }

    void PxAllocatorCallback_deallocate(physx_PxAllocatorCallback* self__pod, void* ptr) {
        physx::PxAllocatorCallback* self_ = reinterpret_cast<physx::PxAllocatorCallback*>(self__pod);
        self_->deallocate(ptr);
    }

    void PxAssertHandler_delete(physx_PxAssertHandler* self__pod) {
        physx::PxAssertHandler* self_ = reinterpret_cast<physx::PxAssertHandler*>(self__pod);
        delete self_;
    }

    physx_PxAssertHandler* phys_PxGetAssertHandler() {
        physx::PxAssertHandler& return_val = PxGetAssertHandler();
        auto return_val_pod = reinterpret_cast<physx_PxAssertHandler*>(&return_val);
        return return_val_pod;
    }

    void phys_PxSetAssertHandler(physx_PxAssertHandler* handler_pod) {
        physx::PxAssertHandler& handler = reinterpret_cast<physx::PxAssertHandler&>(*handler_pod);
        PxSetAssertHandler(handler);
    }

    void PxFoundation_release(physx_PxFoundation* self__pod) {
        physx::PxFoundation* self_ = reinterpret_cast<physx::PxFoundation*>(self__pod);
        self_->release();
    }

    physx_PxErrorCallback* PxFoundation_getErrorCallback(physx_PxFoundation* self__pod) {
        physx::PxFoundation* self_ = reinterpret_cast<physx::PxFoundation*>(self__pod);
        physx::PxErrorCallback& return_val = self_->getErrorCallback();
        auto return_val_pod = reinterpret_cast<physx_PxErrorCallback*>(&return_val);
        return return_val_pod;
    }

    void PxFoundation_setErrorLevel(physx_PxFoundation* self__pod, uint32_t mask) {
        physx::PxFoundation* self_ = reinterpret_cast<physx::PxFoundation*>(self__pod);
        self_->setErrorLevel(mask);
    }

    uint32_t PxFoundation_getErrorLevel(physx_PxFoundation const* self__pod) {
        physx::PxFoundation const* self_ = reinterpret_cast<physx::PxFoundation const*>(self__pod);
        uint32_t return_val = self_->getErrorLevel();
        return return_val;
    }

    physx_PxAllocatorCallback* PxFoundation_getAllocatorCallback(physx_PxFoundation* self__pod) {
        physx::PxFoundation* self_ = reinterpret_cast<physx::PxFoundation*>(self__pod);
        physx::PxAllocatorCallback& return_val = self_->getAllocatorCallback();
        auto return_val_pod = reinterpret_cast<physx_PxAllocatorCallback*>(&return_val);
        return return_val_pod;
    }

    bool PxFoundation_getReportAllocationNames(physx_PxFoundation const* self__pod) {
        physx::PxFoundation const* self_ = reinterpret_cast<physx::PxFoundation const*>(self__pod);
        bool return_val = self_->getReportAllocationNames();
        return return_val;
    }

    void PxFoundation_setReportAllocationNames(physx_PxFoundation* self__pod, bool value) {
        physx::PxFoundation* self_ = reinterpret_cast<physx::PxFoundation*>(self__pod);
        self_->setReportAllocationNames(value);
    }

    void PxFoundation_registerAllocationListener(physx_PxFoundation* self__pod, physx_PxAllocationListener* listener_pod) {
        physx::PxFoundation* self_ = reinterpret_cast<physx::PxFoundation*>(self__pod);
        physx::PxAllocationListener& listener = reinterpret_cast<physx::PxAllocationListener&>(*listener_pod);
        self_->registerAllocationListener(listener);
    }

    void PxFoundation_deregisterAllocationListener(physx_PxFoundation* self__pod, physx_PxAllocationListener* listener_pod) {
        physx::PxFoundation* self_ = reinterpret_cast<physx::PxFoundation*>(self__pod);
        physx::PxAllocationListener& listener = reinterpret_cast<physx::PxAllocationListener&>(*listener_pod);
        self_->deregisterAllocationListener(listener);
    }

    void PxFoundation_registerErrorCallback(physx_PxFoundation* self__pod, physx_PxErrorCallback* callback_pod) {
        physx::PxFoundation* self_ = reinterpret_cast<physx::PxFoundation*>(self__pod);
        physx::PxErrorCallback& callback = reinterpret_cast<physx::PxErrorCallback&>(*callback_pod);
        self_->registerErrorCallback(callback);
    }

    void PxFoundation_deregisterErrorCallback(physx_PxFoundation* self__pod, physx_PxErrorCallback* callback_pod) {
        physx::PxFoundation* self_ = reinterpret_cast<physx::PxFoundation*>(self__pod);
        physx::PxErrorCallback& callback = reinterpret_cast<physx::PxErrorCallback&>(*callback_pod);
        self_->deregisterErrorCallback(callback);
    }

    physx_PxFoundation* phys_PxCreateFoundation(uint32_t version, physx_PxAllocatorCallback* allocator_pod, physx_PxErrorCallback* errorCallback_pod) {
        physx::PxAllocatorCallback& allocator = reinterpret_cast<physx::PxAllocatorCallback&>(*allocator_pod);
        physx::PxErrorCallback& errorCallback = reinterpret_cast<physx::PxErrorCallback&>(*errorCallback_pod);
        physx::PxFoundation* return_val = PxCreateFoundation(version, allocator, errorCallback);
        auto return_val_pod = reinterpret_cast<physx_PxFoundation*>(return_val);
        return return_val_pod;
    }

    void phys_PxSetFoundationInstance(physx_PxFoundation* foundation_pod) {
        physx::PxFoundation& foundation = reinterpret_cast<physx::PxFoundation&>(*foundation_pod);
        PxSetFoundationInstance(foundation);
    }

    physx_PxFoundation* phys_PxGetFoundation() {
        physx::PxFoundation& return_val = PxGetFoundation();
        auto return_val_pod = reinterpret_cast<physx_PxFoundation*>(&return_val);
        return return_val_pod;
    }

    physx_PxProfilerCallback* phys_PxGetProfilerCallback() {
        physx::PxProfilerCallback* return_val = PxGetProfilerCallback();
        auto return_val_pod = reinterpret_cast<physx_PxProfilerCallback*>(return_val);
        return return_val_pod;
    }

    void phys_PxSetProfilerCallback(physx_PxProfilerCallback* profiler_pod) {
        physx::PxProfilerCallback* profiler = reinterpret_cast<physx::PxProfilerCallback*>(profiler_pod);
        PxSetProfilerCallback(profiler);
    }

    physx_PxAllocatorCallback* phys_PxGetAllocatorCallback() {
        physx::PxAllocatorCallback* return_val = PxGetAllocatorCallback();
        auto return_val_pod = reinterpret_cast<physx_PxAllocatorCallback*>(return_val);
        return return_val_pod;
    }

    physx_PxAllocatorCallback* phys_PxGetBroadcastAllocator() {
        physx::PxAllocatorCallback* return_val = PxGetBroadcastAllocator();
        auto return_val_pod = reinterpret_cast<physx_PxAllocatorCallback*>(return_val);
        return return_val_pod;
    }

    physx_PxErrorCallback* phys_PxGetErrorCallback() {
        physx::PxErrorCallback* return_val = PxGetErrorCallback();
        auto return_val_pod = reinterpret_cast<physx_PxErrorCallback*>(return_val);
        return return_val_pod;
    }

    physx_PxErrorCallback* phys_PxGetBroadcastError() {
        physx::PxErrorCallback* return_val = PxGetBroadcastError();
        auto return_val_pod = reinterpret_cast<physx_PxErrorCallback*>(return_val);
        return return_val_pod;
    }

    uint32_t phys_PxGetWarnOnceTimeStamp() {
        uint32_t return_val = PxGetWarnOnceTimeStamp();
        return return_val;
    }

    void phys_PxDecFoundationRefCount() {
        PxDecFoundationRefCount();
    }

    void phys_PxIncFoundationRefCount() {
        PxIncFoundationRefCount();
    }

    physx_PxAllocator PxAllocator_new(char const* anon_param0) {
        PxAllocator return_val(anon_param0);
        physx_PxAllocator return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void* PxAllocator_allocate(physx_PxAllocator* self__pod, size_t size_pod, char const* file, int32_t line) {
        physx::PxAllocator* self_ = reinterpret_cast<physx::PxAllocator*>(self__pod);
        size_t size;
        memcpy(&size, &size_pod, sizeof(size));
        void* return_val = self_->allocate(size, file, line);
        return return_val;
    }

    void PxAllocator_deallocate(physx_PxAllocator* self__pod, void* ptr) {
        physx::PxAllocator* self_ = reinterpret_cast<physx::PxAllocator*>(self__pod);
        self_->deallocate(ptr);
    }

    physx_PxRawAllocator PxRawAllocator_new(char const* anon_param0) {
        PxRawAllocator return_val(anon_param0);
        physx_PxRawAllocator return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void* PxRawAllocator_allocate(physx_PxRawAllocator* self__pod, size_t size_pod, char const* anon_param1, int32_t anon_param2) {
        physx::PxRawAllocator* self_ = reinterpret_cast<physx::PxRawAllocator*>(self__pod);
        size_t size;
        memcpy(&size, &size_pod, sizeof(size));
        void* return_val = self_->allocate(size, anon_param1, anon_param2);
        return return_val;
    }

    void PxRawAllocator_deallocate(physx_PxRawAllocator* self__pod, void* ptr) {
        physx::PxRawAllocator* self_ = reinterpret_cast<physx::PxRawAllocator*>(self__pod);
        self_->deallocate(ptr);
    }

    void PxVirtualAllocatorCallback_delete(physx_PxVirtualAllocatorCallback* self__pod) {
        physx::PxVirtualAllocatorCallback* self_ = reinterpret_cast<physx::PxVirtualAllocatorCallback*>(self__pod);
        delete self_;
    }

    void* PxVirtualAllocatorCallback_allocate(physx_PxVirtualAllocatorCallback* self__pod, size_t size_pod, int32_t group, char const* file, int32_t line) {
        physx::PxVirtualAllocatorCallback* self_ = reinterpret_cast<physx::PxVirtualAllocatorCallback*>(self__pod);
        size_t size;
        memcpy(&size, &size_pod, sizeof(size));
        void* return_val = self_->allocate(size, group, file, line);
        return return_val;
    }

    void PxVirtualAllocatorCallback_deallocate(physx_PxVirtualAllocatorCallback* self__pod, void* ptr) {
        physx::PxVirtualAllocatorCallback* self_ = reinterpret_cast<physx::PxVirtualAllocatorCallback*>(self__pod);
        self_->deallocate(ptr);
    }

    physx_PxVirtualAllocator PxVirtualAllocator_new(physx_PxVirtualAllocatorCallback* callback_pod, int32_t group) {
        physx::PxVirtualAllocatorCallback* callback = reinterpret_cast<physx::PxVirtualAllocatorCallback*>(callback_pod);
        PxVirtualAllocator return_val(callback, group);
        physx_PxVirtualAllocator return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void* PxVirtualAllocator_allocate(physx_PxVirtualAllocator* self__pod, size_t size_pod, char const* file, int32_t line) {
        physx::PxVirtualAllocator* self_ = reinterpret_cast<physx::PxVirtualAllocator*>(self__pod);
        size_t size;
        memcpy(&size, &size_pod, sizeof(size));
        void* return_val = self_->allocate(size, file, line);
        return return_val;
    }

    void PxVirtualAllocator_deallocate(physx_PxVirtualAllocator* self__pod, void* ptr) {
        physx::PxVirtualAllocator* self_ = reinterpret_cast<physx::PxVirtualAllocator*>(self__pod);
        self_->deallocate(ptr);
    }

    physx_PxTempAllocatorChunk PxTempAllocatorChunk_new() {
        PxTempAllocatorChunk return_val;
        physx_PxTempAllocatorChunk return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxTempAllocator PxTempAllocator_new(char const* anon_param0) {
        PxTempAllocator return_val(anon_param0);
        physx_PxTempAllocator return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void* PxTempAllocator_allocate(physx_PxTempAllocator* self__pod, size_t size_pod, char const* file, int32_t line) {
        physx::PxTempAllocator* self_ = reinterpret_cast<physx::PxTempAllocator*>(self__pod);
        size_t size;
        memcpy(&size, &size_pod, sizeof(size));
        void* return_val = self_->allocate(size, file, line);
        return return_val;
    }

    void PxTempAllocator_deallocate(physx_PxTempAllocator* self__pod, void* ptr) {
        physx::PxTempAllocator* self_ = reinterpret_cast<physx::PxTempAllocator*>(self__pod);
        self_->deallocate(ptr);
    }

    void* phys_PxMemZero(void* dest, uint32_t count) {
        void* return_val = PxMemZero(dest, count);
        return return_val;
    }

    void* phys_PxMemSet(void* dest, int32_t c, uint32_t count) {
        void* return_val = PxMemSet(dest, c, count);
        return return_val;
    }

    void* phys_PxMemCopy(void* dest, void const* src, uint32_t count) {
        void* return_val = PxMemCopy(dest, src, count);
        return return_val;
    }

    void* phys_PxMemMove(void* dest, void const* src, uint32_t count) {
        void* return_val = PxMemMove(dest, src, count);
        return return_val;
    }

    void phys_PxMarkSerializedMemory(void* ptr, uint32_t byteSize) {
        PxMarkSerializedMemory(ptr, byteSize);
    }

    void phys_PxMemoryBarrier() {
        PxMemoryBarrier();
    }

    uint32_t phys_PxHighestSetBitUnsafe(uint32_t v) {
        uint32_t return_val = PxHighestSetBitUnsafe(v);
        return return_val;
    }

    uint32_t phys_PxLowestSetBitUnsafe(uint32_t v) {
        uint32_t return_val = PxLowestSetBitUnsafe(v);
        return return_val;
    }

    uint32_t phys_PxCountLeadingZeros(uint32_t v) {
        uint32_t return_val = PxCountLeadingZeros(v);
        return return_val;
    }

    void phys_PxPrefetchLine(void const* ptr, uint32_t offset) {
        PxPrefetchLine(ptr, offset);
    }

    void phys_PxPrefetch(void const* ptr, uint32_t count) {
        PxPrefetch(ptr, count);
    }

    uint32_t phys_PxBitCount(uint32_t v) {
        uint32_t return_val = PxBitCount(v);
        return return_val;
    }

    bool phys_PxIsPowerOfTwo(uint32_t x) {
        bool return_val = PxIsPowerOfTwo(x);
        return return_val;
    }

    uint32_t phys_PxNextPowerOfTwo(uint32_t x) {
        uint32_t return_val = PxNextPowerOfTwo(x);
        return return_val;
    }

    uint32_t phys_PxLowestSetBit(uint32_t x) {
        uint32_t return_val = PxLowestSetBit(x);
        return return_val;
    }

    uint32_t phys_PxHighestSetBit(uint32_t x) {
        uint32_t return_val = PxHighestSetBit(x);
        return return_val;
    }

    uint32_t phys_PxILog2(uint32_t num) {
        uint32_t return_val = PxILog2(num);
        return return_val;
    }

    physx_PxVec3 PxVec3_new() {
        PxVec3 return_val;
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3 PxVec3_new_1(PxZERO anon_param0_pod) {
        auto anon_param0 = static_cast<physx::PxZERO>(anon_param0_pod);
        PxVec3 return_val(anon_param0);
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3 PxVec3_new_2(float a) {
        PxVec3 return_val(a);
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3 PxVec3_new_3(float nx, float ny, float nz) {
        PxVec3 return_val(nx, ny, nz);
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxVec3_isZero(physx_PxVec3 const* self__pod) {
        physx::PxVec3 const* self_ = reinterpret_cast<physx::PxVec3 const*>(self__pod);
        bool return_val = self_->isZero();
        return return_val;
    }

    bool PxVec3_isFinite(physx_PxVec3 const* self__pod) {
        physx::PxVec3 const* self_ = reinterpret_cast<physx::PxVec3 const*>(self__pod);
        bool return_val = self_->isFinite();
        return return_val;
    }

    bool PxVec3_isNormalized(physx_PxVec3 const* self__pod) {
        physx::PxVec3 const* self_ = reinterpret_cast<physx::PxVec3 const*>(self__pod);
        bool return_val = self_->isNormalized();
        return return_val;
    }

    float PxVec3_magnitudeSquared(physx_PxVec3 const* self__pod) {
        physx::PxVec3 const* self_ = reinterpret_cast<physx::PxVec3 const*>(self__pod);
        float return_val = self_->magnitudeSquared();
        return return_val;
    }

    float PxVec3_magnitude(physx_PxVec3 const* self__pod) {
        physx::PxVec3 const* self_ = reinterpret_cast<physx::PxVec3 const*>(self__pod);
        float return_val = self_->magnitude();
        return return_val;
    }

    float PxVec3_dot(physx_PxVec3 const* self__pod, physx_PxVec3 const* v_pod) {
        physx::PxVec3 const* self_ = reinterpret_cast<physx::PxVec3 const*>(self__pod);
        physx::PxVec3 const& v = reinterpret_cast<physx::PxVec3 const&>(*v_pod);
        float return_val = self_->dot(v);
        return return_val;
    }

    physx_PxVec3 PxVec3_cross(physx_PxVec3 const* self__pod, physx_PxVec3 const* v_pod) {
        physx::PxVec3 const* self_ = reinterpret_cast<physx::PxVec3 const*>(self__pod);
        physx::PxVec3 const& v = reinterpret_cast<physx::PxVec3 const&>(*v_pod);
        physx::PxVec3 return_val = self_->cross(v);
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3 PxVec3_getNormalized(physx_PxVec3 const* self__pod) {
        physx::PxVec3 const* self_ = reinterpret_cast<physx::PxVec3 const*>(self__pod);
        physx::PxVec3 return_val = self_->getNormalized();
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    float PxVec3_normalize(physx_PxVec3* self__pod) {
        physx::PxVec3* self_ = reinterpret_cast<physx::PxVec3*>(self__pod);
        float return_val = self_->normalize();
        return return_val;
    }

    float PxVec3_normalizeSafe(physx_PxVec3* self__pod) {
        physx::PxVec3* self_ = reinterpret_cast<physx::PxVec3*>(self__pod);
        float return_val = self_->normalizeSafe();
        return return_val;
    }

    float PxVec3_normalizeFast(physx_PxVec3* self__pod) {
        physx::PxVec3* self_ = reinterpret_cast<physx::PxVec3*>(self__pod);
        float return_val = self_->normalizeFast();
        return return_val;
    }

    physx_PxVec3 PxVec3_multiply(physx_PxVec3 const* self__pod, physx_PxVec3 const* a_pod) {
        physx::PxVec3 const* self_ = reinterpret_cast<physx::PxVec3 const*>(self__pod);
        physx::PxVec3 const& a = reinterpret_cast<physx::PxVec3 const&>(*a_pod);
        physx::PxVec3 return_val = self_->multiply(a);
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3 PxVec3_minimum(physx_PxVec3 const* self__pod, physx_PxVec3 const* v_pod) {
        physx::PxVec3 const* self_ = reinterpret_cast<physx::PxVec3 const*>(self__pod);
        physx::PxVec3 const& v = reinterpret_cast<physx::PxVec3 const&>(*v_pod);
        physx::PxVec3 return_val = self_->minimum(v);
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    float PxVec3_minElement(physx_PxVec3 const* self__pod) {
        physx::PxVec3 const* self_ = reinterpret_cast<physx::PxVec3 const*>(self__pod);
        float return_val = self_->minElement();
        return return_val;
    }

    physx_PxVec3 PxVec3_maximum(physx_PxVec3 const* self__pod, physx_PxVec3 const* v_pod) {
        physx::PxVec3 const* self_ = reinterpret_cast<physx::PxVec3 const*>(self__pod);
        physx::PxVec3 const& v = reinterpret_cast<physx::PxVec3 const&>(*v_pod);
        physx::PxVec3 return_val = self_->maximum(v);
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    float PxVec3_maxElement(physx_PxVec3 const* self__pod) {
        physx::PxVec3 const* self_ = reinterpret_cast<physx::PxVec3 const*>(self__pod);
        float return_val = self_->maxElement();
        return return_val;
    }

    physx_PxVec3 PxVec3_abs(physx_PxVec3 const* self__pod) {
        physx::PxVec3 const* self_ = reinterpret_cast<physx::PxVec3 const*>(self__pod);
        physx::PxVec3 return_val = self_->abs();
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3Padded* PxVec3Padded_new_alloc() {
        auto return_val = new physx::PxVec3Padded();
        auto return_val_pod = reinterpret_cast<physx_PxVec3Padded*>(return_val);
        return return_val_pod;
    }

    void PxVec3Padded_delete(physx_PxVec3Padded* self__pod) {
        physx::PxVec3Padded* self_ = reinterpret_cast<physx::PxVec3Padded*>(self__pod);
        delete self_;
    }

    physx_PxVec3Padded* PxVec3Padded_new_alloc_1(physx_PxVec3 const* p_pod) {
        physx::PxVec3 const& p = reinterpret_cast<physx::PxVec3 const&>(*p_pod);
        auto return_val = new physx::PxVec3Padded(p);
        auto return_val_pod = reinterpret_cast<physx_PxVec3Padded*>(return_val);
        return return_val_pod;
    }

    physx_PxVec3Padded* PxVec3Padded_new_alloc_2(float f) {
        auto return_val = new physx::PxVec3Padded(f);
        auto return_val_pod = reinterpret_cast<physx_PxVec3Padded*>(return_val);
        return return_val_pod;
    }

    physx_PxQuat PxQuat_new() {
        PxQuat return_val;
        physx_PxQuat return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxQuat PxQuat_new_1(PxIDENTITY anon_param0_pod) {
        auto anon_param0 = static_cast<physx::PxIDENTITY>(anon_param0_pod);
        PxQuat return_val(anon_param0);
        physx_PxQuat return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxQuat PxQuat_new_2(float r) {
        PxQuat return_val(r);
        physx_PxQuat return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxQuat PxQuat_new_3(float nx, float ny, float nz, float nw) {
        PxQuat return_val(nx, ny, nz, nw);
        physx_PxQuat return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxQuat PxQuat_new_4(float angleRadians, physx_PxVec3 const* unitAxis_pod) {
        physx::PxVec3 const& unitAxis = reinterpret_cast<physx::PxVec3 const&>(*unitAxis_pod);
        PxQuat return_val(angleRadians, unitAxis);
        physx_PxQuat return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxQuat PxQuat_new_5(physx_PxMat33 const* m_pod) {
        physx::PxMat33 const& m = reinterpret_cast<physx::PxMat33 const&>(*m_pod);
        PxQuat return_val(m);
        physx_PxQuat return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxQuat_isIdentity(physx_PxQuat const* self__pod) {
        physx::PxQuat const* self_ = reinterpret_cast<physx::PxQuat const*>(self__pod);
        bool return_val = self_->isIdentity();
        return return_val;
    }

    bool PxQuat_isFinite(physx_PxQuat const* self__pod) {
        physx::PxQuat const* self_ = reinterpret_cast<physx::PxQuat const*>(self__pod);
        bool return_val = self_->isFinite();
        return return_val;
    }

    bool PxQuat_isUnit(physx_PxQuat const* self__pod) {
        physx::PxQuat const* self_ = reinterpret_cast<physx::PxQuat const*>(self__pod);
        bool return_val = self_->isUnit();
        return return_val;
    }

    bool PxQuat_isSane(physx_PxQuat const* self__pod) {
        physx::PxQuat const* self_ = reinterpret_cast<physx::PxQuat const*>(self__pod);
        bool return_val = self_->isSane();
        return return_val;
    }

    void PxQuat_toRadiansAndUnitAxis(physx_PxQuat const* self__pod, float* angle_pod, physx_PxVec3* axis_pod) {
        physx::PxQuat const* self_ = reinterpret_cast<physx::PxQuat const*>(self__pod);
        float& angle = *angle_pod;
        physx::PxVec3& axis = reinterpret_cast<physx::PxVec3&>(*axis_pod);
        self_->toRadiansAndUnitAxis(angle, axis);
    }

    float PxQuat_getAngle(physx_PxQuat const* self__pod) {
        physx::PxQuat const* self_ = reinterpret_cast<physx::PxQuat const*>(self__pod);
        float return_val = self_->getAngle();
        return return_val;
    }

    float PxQuat_getAngle_1(physx_PxQuat const* self__pod, physx_PxQuat const* q_pod) {
        physx::PxQuat const* self_ = reinterpret_cast<physx::PxQuat const*>(self__pod);
        physx::PxQuat const& q = reinterpret_cast<physx::PxQuat const&>(*q_pod);
        float return_val = self_->getAngle(q);
        return return_val;
    }

    float PxQuat_magnitudeSquared(physx_PxQuat const* self__pod) {
        physx::PxQuat const* self_ = reinterpret_cast<physx::PxQuat const*>(self__pod);
        float return_val = self_->magnitudeSquared();
        return return_val;
    }

    float PxQuat_dot(physx_PxQuat const* self__pod, physx_PxQuat const* v_pod) {
        physx::PxQuat const* self_ = reinterpret_cast<physx::PxQuat const*>(self__pod);
        physx::PxQuat const& v = reinterpret_cast<physx::PxQuat const&>(*v_pod);
        float return_val = self_->dot(v);
        return return_val;
    }

    physx_PxQuat PxQuat_getNormalized(physx_PxQuat const* self__pod) {
        physx::PxQuat const* self_ = reinterpret_cast<physx::PxQuat const*>(self__pod);
        physx::PxQuat return_val = self_->getNormalized();
        physx_PxQuat return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    float PxQuat_magnitude(physx_PxQuat const* self__pod) {
        physx::PxQuat const* self_ = reinterpret_cast<physx::PxQuat const*>(self__pod);
        float return_val = self_->magnitude();
        return return_val;
    }

    float PxQuat_normalize(physx_PxQuat* self__pod) {
        physx::PxQuat* self_ = reinterpret_cast<physx::PxQuat*>(self__pod);
        float return_val = self_->normalize();
        return return_val;
    }

    physx_PxQuat PxQuat_getConjugate(physx_PxQuat const* self__pod) {
        physx::PxQuat const* self_ = reinterpret_cast<physx::PxQuat const*>(self__pod);
        physx::PxQuat return_val = self_->getConjugate();
        physx_PxQuat return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3 PxQuat_getImaginaryPart(physx_PxQuat const* self__pod) {
        physx::PxQuat const* self_ = reinterpret_cast<physx::PxQuat const*>(self__pod);
        physx::PxVec3 return_val = self_->getImaginaryPart();
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3 PxQuat_getBasisVector0(physx_PxQuat const* self__pod) {
        physx::PxQuat const* self_ = reinterpret_cast<physx::PxQuat const*>(self__pod);
        physx::PxVec3 return_val = self_->getBasisVector0();
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3 PxQuat_getBasisVector1(physx_PxQuat const* self__pod) {
        physx::PxQuat const* self_ = reinterpret_cast<physx::PxQuat const*>(self__pod);
        physx::PxVec3 return_val = self_->getBasisVector1();
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3 PxQuat_getBasisVector2(physx_PxQuat const* self__pod) {
        physx::PxQuat const* self_ = reinterpret_cast<physx::PxQuat const*>(self__pod);
        physx::PxVec3 return_val = self_->getBasisVector2();
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3 PxQuat_rotate(physx_PxQuat const* self__pod, physx_PxVec3 const* v_pod) {
        physx::PxQuat const* self_ = reinterpret_cast<physx::PxQuat const*>(self__pod);
        physx::PxVec3 const& v = reinterpret_cast<physx::PxVec3 const&>(*v_pod);
        physx::PxVec3 return_val = self_->rotate(v);
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3 PxQuat_rotateInv(physx_PxQuat const* self__pod, physx_PxVec3 const* v_pod) {
        physx::PxQuat const* self_ = reinterpret_cast<physx::PxQuat const*>(self__pod);
        physx::PxVec3 const& v = reinterpret_cast<physx::PxVec3 const&>(*v_pod);
        physx::PxVec3 return_val = self_->rotateInv(v);
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxTransform PxTransform_new() {
        PxTransform return_val;
        physx_PxTransform return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxTransform PxTransform_new_1(physx_PxVec3 const* position_pod) {
        physx::PxVec3 const& position = reinterpret_cast<physx::PxVec3 const&>(*position_pod);
        PxTransform return_val(position);
        physx_PxTransform return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxTransform PxTransform_new_2(PxIDENTITY anon_param0_pod) {
        auto anon_param0 = static_cast<physx::PxIDENTITY>(anon_param0_pod);
        PxTransform return_val(anon_param0);
        physx_PxTransform return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxTransform PxTransform_new_3(physx_PxQuat const* orientation_pod) {
        physx::PxQuat const& orientation = reinterpret_cast<physx::PxQuat const&>(*orientation_pod);
        PxTransform return_val(orientation);
        physx_PxTransform return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxTransform PxTransform_new_4(float x, float y, float z, physx_PxQuat aQ_pod) {
        physx::PxQuat aQ;
        memcpy(&aQ, &aQ_pod, sizeof(aQ));
        PxTransform return_val(x, y, z, aQ);
        physx_PxTransform return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxTransform PxTransform_new_5(physx_PxVec3 const* p0_pod, physx_PxQuat const* q0_pod) {
        physx::PxVec3 const& p0 = reinterpret_cast<physx::PxVec3 const&>(*p0_pod);
        physx::PxQuat const& q0 = reinterpret_cast<physx::PxQuat const&>(*q0_pod);
        PxTransform return_val(p0, q0);
        physx_PxTransform return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxTransform PxTransform_new_6(physx_PxMat44 const* m_pod) {
        physx::PxMat44 const& m = reinterpret_cast<physx::PxMat44 const&>(*m_pod);
        PxTransform return_val(m);
        physx_PxTransform return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxTransform PxTransform_getInverse(physx_PxTransform const* self__pod) {
        physx::PxTransform const* self_ = reinterpret_cast<physx::PxTransform const*>(self__pod);
        physx::PxTransform return_val = self_->getInverse();
        physx_PxTransform return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3 PxTransform_transform(physx_PxTransform const* self__pod, physx_PxVec3 const* input_pod) {
        physx::PxTransform const* self_ = reinterpret_cast<physx::PxTransform const*>(self__pod);
        physx::PxVec3 const& input = reinterpret_cast<physx::PxVec3 const&>(*input_pod);
        physx::PxVec3 return_val = self_->transform(input);
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3 PxTransform_transformInv(physx_PxTransform const* self__pod, physx_PxVec3 const* input_pod) {
        physx::PxTransform const* self_ = reinterpret_cast<physx::PxTransform const*>(self__pod);
        physx::PxVec3 const& input = reinterpret_cast<physx::PxVec3 const&>(*input_pod);
        physx::PxVec3 return_val = self_->transformInv(input);
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3 PxTransform_rotate(physx_PxTransform const* self__pod, physx_PxVec3 const* input_pod) {
        physx::PxTransform const* self_ = reinterpret_cast<physx::PxTransform const*>(self__pod);
        physx::PxVec3 const& input = reinterpret_cast<physx::PxVec3 const&>(*input_pod);
        physx::PxVec3 return_val = self_->rotate(input);
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3 PxTransform_rotateInv(physx_PxTransform const* self__pod, physx_PxVec3 const* input_pod) {
        physx::PxTransform const* self_ = reinterpret_cast<physx::PxTransform const*>(self__pod);
        physx::PxVec3 const& input = reinterpret_cast<physx::PxVec3 const&>(*input_pod);
        physx::PxVec3 return_val = self_->rotateInv(input);
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxTransform PxTransform_transform_1(physx_PxTransform const* self__pod, physx_PxTransform const* src_pod) {
        physx::PxTransform const* self_ = reinterpret_cast<physx::PxTransform const*>(self__pod);
        physx::PxTransform const& src = reinterpret_cast<physx::PxTransform const&>(*src_pod);
        physx::PxTransform return_val = self_->transform(src);
        physx_PxTransform return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxTransform_isValid(physx_PxTransform const* self__pod) {
        physx::PxTransform const* self_ = reinterpret_cast<physx::PxTransform const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    bool PxTransform_isSane(physx_PxTransform const* self__pod) {
        physx::PxTransform const* self_ = reinterpret_cast<physx::PxTransform const*>(self__pod);
        bool return_val = self_->isSane();
        return return_val;
    }

    bool PxTransform_isFinite(physx_PxTransform const* self__pod) {
        physx::PxTransform const* self_ = reinterpret_cast<physx::PxTransform const*>(self__pod);
        bool return_val = self_->isFinite();
        return return_val;
    }

    physx_PxTransform PxTransform_transformInv_1(physx_PxTransform const* self__pod, physx_PxTransform const* src_pod) {
        physx::PxTransform const* self_ = reinterpret_cast<physx::PxTransform const*>(self__pod);
        physx::PxTransform const& src = reinterpret_cast<physx::PxTransform const&>(*src_pod);
        physx::PxTransform return_val = self_->transformInv(src);
        physx_PxTransform return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxTransform PxTransform_getNormalized(physx_PxTransform const* self__pod) {
        physx::PxTransform const* self_ = reinterpret_cast<physx::PxTransform const*>(self__pod);
        physx::PxTransform return_val = self_->getNormalized();
        physx_PxTransform return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxMat33 PxMat33_new() {
        PxMat33 return_val;
        physx_PxMat33 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxMat33 PxMat33_new_1(PxIDENTITY anon_param0_pod) {
        auto anon_param0 = static_cast<physx::PxIDENTITY>(anon_param0_pod);
        PxMat33 return_val(anon_param0);
        physx_PxMat33 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxMat33 PxMat33_new_2(PxZERO anon_param0_pod) {
        auto anon_param0 = static_cast<physx::PxZERO>(anon_param0_pod);
        PxMat33 return_val(anon_param0);
        physx_PxMat33 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxMat33 PxMat33_new_3(physx_PxVec3 const* col0_pod, physx_PxVec3 const* col1_pod, physx_PxVec3 const* col2_pod) {
        physx::PxVec3 const& col0 = reinterpret_cast<physx::PxVec3 const&>(*col0_pod);
        physx::PxVec3 const& col1 = reinterpret_cast<physx::PxVec3 const&>(*col1_pod);
        physx::PxVec3 const& col2 = reinterpret_cast<physx::PxVec3 const&>(*col2_pod);
        PxMat33 return_val(col0, col1, col2);
        physx_PxMat33 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxMat33 PxMat33_new_4(float r) {
        PxMat33 return_val(r);
        physx_PxMat33 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxMat33 PxMat33_new_5(float* values) {
        PxMat33 return_val(values);
        physx_PxMat33 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxMat33 PxMat33_new_6(physx_PxQuat const* q_pod) {
        physx::PxQuat const& q = reinterpret_cast<physx::PxQuat const&>(*q_pod);
        PxMat33 return_val(q);
        physx_PxMat33 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxMat33 PxMat33_createDiagonal(physx_PxVec3 const* d_pod) {
        physx::PxVec3 const& d = reinterpret_cast<physx::PxVec3 const&>(*d_pod);
        physx::PxMat33 return_val = PxMat33::createDiagonal(d);
        physx_PxMat33 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxMat33 PxMat33_outer(physx_PxVec3 const* a_pod, physx_PxVec3 const* b_pod) {
        physx::PxVec3 const& a = reinterpret_cast<physx::PxVec3 const&>(*a_pod);
        physx::PxVec3 const& b = reinterpret_cast<physx::PxVec3 const&>(*b_pod);
        physx::PxMat33 return_val = PxMat33::outer(a, b);
        physx_PxMat33 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxMat33 PxMat33_getTranspose(physx_PxMat33 const* self__pod) {
        physx::PxMat33 const* self_ = reinterpret_cast<physx::PxMat33 const*>(self__pod);
        physx::PxMat33 return_val = self_->getTranspose();
        physx_PxMat33 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxMat33 PxMat33_getInverse(physx_PxMat33 const* self__pod) {
        physx::PxMat33 const* self_ = reinterpret_cast<physx::PxMat33 const*>(self__pod);
        physx::PxMat33 return_val = self_->getInverse();
        physx_PxMat33 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    float PxMat33_getDeterminant(physx_PxMat33 const* self__pod) {
        physx::PxMat33 const* self_ = reinterpret_cast<physx::PxMat33 const*>(self__pod);
        float return_val = self_->getDeterminant();
        return return_val;
    }

    physx_PxVec3 PxMat33_transform(physx_PxMat33 const* self__pod, physx_PxVec3 const* other_pod) {
        physx::PxMat33 const* self_ = reinterpret_cast<physx::PxMat33 const*>(self__pod);
        physx::PxVec3 const& other = reinterpret_cast<physx::PxVec3 const&>(*other_pod);
        physx::PxVec3 return_val = self_->transform(other);
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3 PxMat33_transformTranspose(physx_PxMat33 const* self__pod, physx_PxVec3 const* other_pod) {
        physx::PxMat33 const* self_ = reinterpret_cast<physx::PxMat33 const*>(self__pod);
        physx::PxVec3 const& other = reinterpret_cast<physx::PxVec3 const&>(*other_pod);
        physx::PxVec3 return_val = self_->transformTranspose(other);
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    float const* PxMat33_front(physx_PxMat33 const* self__pod) {
        physx::PxMat33 const* self_ = reinterpret_cast<physx::PxMat33 const*>(self__pod);
        float const* return_val = self_->front();
        return return_val;
    }

    physx_PxBounds3 PxBounds3_new() {
        PxBounds3 return_val;
        physx_PxBounds3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxBounds3 PxBounds3_new_1(physx_PxVec3 const* minimum_pod, physx_PxVec3 const* maximum_pod) {
        physx::PxVec3 const& minimum = reinterpret_cast<physx::PxVec3 const&>(*minimum_pod);
        physx::PxVec3 const& maximum = reinterpret_cast<physx::PxVec3 const&>(*maximum_pod);
        PxBounds3 return_val(minimum, maximum);
        physx_PxBounds3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxBounds3 PxBounds3_empty() {
        physx::PxBounds3 return_val = PxBounds3::empty();
        physx_PxBounds3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxBounds3 PxBounds3_boundsOfPoints(physx_PxVec3 const* v0_pod, physx_PxVec3 const* v1_pod) {
        physx::PxVec3 const& v0 = reinterpret_cast<physx::PxVec3 const&>(*v0_pod);
        physx::PxVec3 const& v1 = reinterpret_cast<physx::PxVec3 const&>(*v1_pod);
        physx::PxBounds3 return_val = PxBounds3::boundsOfPoints(v0, v1);
        physx_PxBounds3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxBounds3 PxBounds3_centerExtents(physx_PxVec3 const* center_pod, physx_PxVec3 const* extent_pod) {
        physx::PxVec3 const& center = reinterpret_cast<physx::PxVec3 const&>(*center_pod);
        physx::PxVec3 const& extent = reinterpret_cast<physx::PxVec3 const&>(*extent_pod);
        physx::PxBounds3 return_val = PxBounds3::centerExtents(center, extent);
        physx_PxBounds3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxBounds3 PxBounds3_basisExtent(physx_PxVec3 const* center_pod, physx_PxMat33 const* basis_pod, physx_PxVec3 const* extent_pod) {
        physx::PxVec3 const& center = reinterpret_cast<physx::PxVec3 const&>(*center_pod);
        physx::PxMat33 const& basis = reinterpret_cast<physx::PxMat33 const&>(*basis_pod);
        physx::PxVec3 const& extent = reinterpret_cast<physx::PxVec3 const&>(*extent_pod);
        physx::PxBounds3 return_val = PxBounds3::basisExtent(center, basis, extent);
        physx_PxBounds3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxBounds3 PxBounds3_poseExtent(physx_PxTransform const* pose_pod, physx_PxVec3 const* extent_pod) {
        physx::PxTransform const& pose = reinterpret_cast<physx::PxTransform const&>(*pose_pod);
        physx::PxVec3 const& extent = reinterpret_cast<physx::PxVec3 const&>(*extent_pod);
        physx::PxBounds3 return_val = PxBounds3::poseExtent(pose, extent);
        physx_PxBounds3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxBounds3 PxBounds3_transformSafe(physx_PxMat33 const* matrix_pod, physx_PxBounds3 const* bounds_pod) {
        physx::PxMat33 const& matrix = reinterpret_cast<physx::PxMat33 const&>(*matrix_pod);
        physx::PxBounds3 const& bounds = reinterpret_cast<physx::PxBounds3 const&>(*bounds_pod);
        physx::PxBounds3 return_val = PxBounds3::transformSafe(matrix, bounds);
        physx_PxBounds3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxBounds3 PxBounds3_transformFast(physx_PxMat33 const* matrix_pod, physx_PxBounds3 const* bounds_pod) {
        physx::PxMat33 const& matrix = reinterpret_cast<physx::PxMat33 const&>(*matrix_pod);
        physx::PxBounds3 const& bounds = reinterpret_cast<physx::PxBounds3 const&>(*bounds_pod);
        physx::PxBounds3 return_val = PxBounds3::transformFast(matrix, bounds);
        physx_PxBounds3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxBounds3 PxBounds3_transformSafe_1(physx_PxTransform const* transform_pod, physx_PxBounds3 const* bounds_pod) {
        physx::PxTransform const& transform = reinterpret_cast<physx::PxTransform const&>(*transform_pod);
        physx::PxBounds3 const& bounds = reinterpret_cast<physx::PxBounds3 const&>(*bounds_pod);
        physx::PxBounds3 return_val = PxBounds3::transformSafe(transform, bounds);
        physx_PxBounds3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxBounds3 PxBounds3_transformFast_1(physx_PxTransform const* transform_pod, physx_PxBounds3 const* bounds_pod) {
        physx::PxTransform const& transform = reinterpret_cast<physx::PxTransform const&>(*transform_pod);
        physx::PxBounds3 const& bounds = reinterpret_cast<physx::PxBounds3 const&>(*bounds_pod);
        physx::PxBounds3 return_val = PxBounds3::transformFast(transform, bounds);
        physx_PxBounds3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxBounds3_setEmpty(physx_PxBounds3* self__pod) {
        physx::PxBounds3* self_ = reinterpret_cast<physx::PxBounds3*>(self__pod);
        self_->setEmpty();
    }

    void PxBounds3_setMaximal(physx_PxBounds3* self__pod) {
        physx::PxBounds3* self_ = reinterpret_cast<physx::PxBounds3*>(self__pod);
        self_->setMaximal();
    }

    void PxBounds3_include(physx_PxBounds3* self__pod, physx_PxVec3 const* v_pod) {
        physx::PxBounds3* self_ = reinterpret_cast<physx::PxBounds3*>(self__pod);
        physx::PxVec3 const& v = reinterpret_cast<physx::PxVec3 const&>(*v_pod);
        self_->include(v);
    }

    void PxBounds3_include_1(physx_PxBounds3* self__pod, physx_PxBounds3 const* b_pod) {
        physx::PxBounds3* self_ = reinterpret_cast<physx::PxBounds3*>(self__pod);
        physx::PxBounds3 const& b = reinterpret_cast<physx::PxBounds3 const&>(*b_pod);
        self_->include(b);
    }

    bool PxBounds3_isEmpty(physx_PxBounds3 const* self__pod) {
        physx::PxBounds3 const* self_ = reinterpret_cast<physx::PxBounds3 const*>(self__pod);
        bool return_val = self_->isEmpty();
        return return_val;
    }

    bool PxBounds3_intersects(physx_PxBounds3 const* self__pod, physx_PxBounds3 const* b_pod) {
        physx::PxBounds3 const* self_ = reinterpret_cast<physx::PxBounds3 const*>(self__pod);
        physx::PxBounds3 const& b = reinterpret_cast<physx::PxBounds3 const&>(*b_pod);
        bool return_val = self_->intersects(b);
        return return_val;
    }

    bool PxBounds3_intersects1D(physx_PxBounds3 const* self__pod, physx_PxBounds3 const* a_pod, uint32_t axis) {
        physx::PxBounds3 const* self_ = reinterpret_cast<physx::PxBounds3 const*>(self__pod);
        physx::PxBounds3 const& a = reinterpret_cast<physx::PxBounds3 const&>(*a_pod);
        bool return_val = self_->intersects1D(a, axis);
        return return_val;
    }

    bool PxBounds3_contains(physx_PxBounds3 const* self__pod, physx_PxVec3 const* v_pod) {
        physx::PxBounds3 const* self_ = reinterpret_cast<physx::PxBounds3 const*>(self__pod);
        physx::PxVec3 const& v = reinterpret_cast<physx::PxVec3 const&>(*v_pod);
        bool return_val = self_->contains(v);
        return return_val;
    }

    bool PxBounds3_isInside(physx_PxBounds3 const* self__pod, physx_PxBounds3 const* box_pod) {
        physx::PxBounds3 const* self_ = reinterpret_cast<physx::PxBounds3 const*>(self__pod);
        physx::PxBounds3 const& box = reinterpret_cast<physx::PxBounds3 const&>(*box_pod);
        bool return_val = self_->isInside(box);
        return return_val;
    }

    physx_PxVec3 PxBounds3_getCenter(physx_PxBounds3 const* self__pod) {
        physx::PxBounds3 const* self_ = reinterpret_cast<physx::PxBounds3 const*>(self__pod);
        physx::PxVec3 return_val = self_->getCenter();
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    float PxBounds3_getCenter_1(physx_PxBounds3 const* self__pod, uint32_t axis) {
        physx::PxBounds3 const* self_ = reinterpret_cast<physx::PxBounds3 const*>(self__pod);
        float return_val = self_->getCenter(axis);
        return return_val;
    }

    float PxBounds3_getExtents(physx_PxBounds3 const* self__pod, uint32_t axis) {
        physx::PxBounds3 const* self_ = reinterpret_cast<physx::PxBounds3 const*>(self__pod);
        float return_val = self_->getExtents(axis);
        return return_val;
    }

    physx_PxVec3 PxBounds3_getDimensions(physx_PxBounds3 const* self__pod) {
        physx::PxBounds3 const* self_ = reinterpret_cast<physx::PxBounds3 const*>(self__pod);
        physx::PxVec3 return_val = self_->getDimensions();
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3 PxBounds3_getExtents_1(physx_PxBounds3 const* self__pod) {
        physx::PxBounds3 const* self_ = reinterpret_cast<physx::PxBounds3 const*>(self__pod);
        physx::PxVec3 return_val = self_->getExtents();
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxBounds3_scaleSafe(physx_PxBounds3* self__pod, float scale) {
        physx::PxBounds3* self_ = reinterpret_cast<physx::PxBounds3*>(self__pod);
        self_->scaleSafe(scale);
    }

    void PxBounds3_scaleFast(physx_PxBounds3* self__pod, float scale) {
        physx::PxBounds3* self_ = reinterpret_cast<physx::PxBounds3*>(self__pod);
        self_->scaleFast(scale);
    }

    void PxBounds3_fattenSafe(physx_PxBounds3* self__pod, float distance) {
        physx::PxBounds3* self_ = reinterpret_cast<physx::PxBounds3*>(self__pod);
        self_->fattenSafe(distance);
    }

    void PxBounds3_fattenFast(physx_PxBounds3* self__pod, float distance) {
        physx::PxBounds3* self_ = reinterpret_cast<physx::PxBounds3*>(self__pod);
        self_->fattenFast(distance);
    }

    bool PxBounds3_isFinite(physx_PxBounds3 const* self__pod) {
        physx::PxBounds3 const* self_ = reinterpret_cast<physx::PxBounds3 const*>(self__pod);
        bool return_val = self_->isFinite();
        return return_val;
    }

    bool PxBounds3_isValid(physx_PxBounds3 const* self__pod) {
        physx::PxBounds3 const* self_ = reinterpret_cast<physx::PxBounds3 const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    physx_PxVec3 PxBounds3_closestPoint(physx_PxBounds3 const* self__pod, physx_PxVec3 const* p_pod) {
        physx::PxBounds3 const* self_ = reinterpret_cast<physx::PxBounds3 const*>(self__pod);
        physx::PxVec3 const& p = reinterpret_cast<physx::PxVec3 const&>(*p_pod);
        physx::PxVec3 return_val = self_->closestPoint(p);
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxErrorCallback_delete(physx_PxErrorCallback* self__pod) {
        physx::PxErrorCallback* self_ = reinterpret_cast<physx::PxErrorCallback*>(self__pod);
        delete self_;
    }

    void PxErrorCallback_reportError(physx_PxErrorCallback* self__pod, PxErrorCode code_pod, char const* message, char const* file, int32_t line) {
        physx::PxErrorCallback* self_ = reinterpret_cast<physx::PxErrorCallback*>(self__pod);
        auto code = static_cast<physx::PxErrorCode::Enum>(code_pod);
        self_->reportError(code, message, file, line);
    }

    void PxAllocationListener_onAllocation(physx_PxAllocationListener* self__pod, size_t size_pod, char const* typeName, char const* filename, int32_t line, void* allocatedMemory) {
        physx::PxAllocationListener* self_ = reinterpret_cast<physx::PxAllocationListener*>(self__pod);
        size_t size;
        memcpy(&size, &size_pod, sizeof(size));
        self_->onAllocation(size, typeName, filename, line, allocatedMemory);
    }

    void PxAllocationListener_onDeallocation(physx_PxAllocationListener* self__pod, void* allocatedMemory) {
        physx::PxAllocationListener* self_ = reinterpret_cast<physx::PxAllocationListener*>(self__pod);
        self_->onDeallocation(allocatedMemory);
    }

    physx_PxBroadcastingAllocator* PxBroadcastingAllocator_new_alloc(physx_PxAllocatorCallback* allocator_pod, physx_PxErrorCallback* error_pod) {
        physx::PxAllocatorCallback& allocator = reinterpret_cast<physx::PxAllocatorCallback&>(*allocator_pod);
        physx::PxErrorCallback& error = reinterpret_cast<physx::PxErrorCallback&>(*error_pod);
        auto return_val = new physx::PxBroadcastingAllocator(allocator, error);
        auto return_val_pod = reinterpret_cast<physx_PxBroadcastingAllocator*>(return_val);
        return return_val_pod;
    }

    void PxBroadcastingAllocator_delete(physx_PxBroadcastingAllocator* self__pod) {
        physx::PxBroadcastingAllocator* self_ = reinterpret_cast<physx::PxBroadcastingAllocator*>(self__pod);
        delete self_;
    }

    void* PxBroadcastingAllocator_allocate(physx_PxBroadcastingAllocator* self__pod, size_t size_pod, char const* typeName, char const* filename, int32_t line) {
        physx::PxBroadcastingAllocator* self_ = reinterpret_cast<physx::PxBroadcastingAllocator*>(self__pod);
        size_t size;
        memcpy(&size, &size_pod, sizeof(size));
        void* return_val = self_->allocate(size, typeName, filename, line);
        return return_val;
    }

    void PxBroadcastingAllocator_deallocate(physx_PxBroadcastingAllocator* self__pod, void* ptr) {
        physx::PxBroadcastingAllocator* self_ = reinterpret_cast<physx::PxBroadcastingAllocator*>(self__pod);
        self_->deallocate(ptr);
    }

    physx_PxBroadcastingErrorCallback* PxBroadcastingErrorCallback_new_alloc(physx_PxErrorCallback* errorCallback_pod) {
        physx::PxErrorCallback& errorCallback = reinterpret_cast<physx::PxErrorCallback&>(*errorCallback_pod);
        auto return_val = new physx::PxBroadcastingErrorCallback(errorCallback);
        auto return_val_pod = reinterpret_cast<physx_PxBroadcastingErrorCallback*>(return_val);
        return return_val_pod;
    }

    void PxBroadcastingErrorCallback_delete(physx_PxBroadcastingErrorCallback* self__pod) {
        physx::PxBroadcastingErrorCallback* self_ = reinterpret_cast<physx::PxBroadcastingErrorCallback*>(self__pod);
        delete self_;
    }

    void PxBroadcastingErrorCallback_reportError(physx_PxBroadcastingErrorCallback* self__pod, PxErrorCode code_pod, char const* message, char const* file, int32_t line) {
        physx::PxBroadcastingErrorCallback* self_ = reinterpret_cast<physx::PxBroadcastingErrorCallback*>(self__pod);
        auto code = static_cast<physx::PxErrorCode::Enum>(code_pod);
        self_->reportError(code, message, file, line);
    }

    void phys_PxEnableFPExceptions() {
        PxEnableFPExceptions();
    }

    void phys_PxDisableFPExceptions() {
        PxDisableFPExceptions();
    }

    uint32_t PxInputStream_read(physx_PxInputStream* self__pod, void* dest, uint32_t count) {
        physx::PxInputStream* self_ = reinterpret_cast<physx::PxInputStream*>(self__pod);
        uint32_t return_val = self_->read(dest, count);
        return return_val;
    }

    void PxInputStream_delete(physx_PxInputStream* self__pod) {
        physx::PxInputStream* self_ = reinterpret_cast<physx::PxInputStream*>(self__pod);
        delete self_;
    }

    uint32_t PxInputData_getLength(physx_PxInputData const* self__pod) {
        physx::PxInputData const* self_ = reinterpret_cast<physx::PxInputData const*>(self__pod);
        uint32_t return_val = self_->getLength();
        return return_val;
    }

    void PxInputData_seek(physx_PxInputData* self__pod, uint32_t offset) {
        physx::PxInputData* self_ = reinterpret_cast<physx::PxInputData*>(self__pod);
        self_->seek(offset);
    }

    uint32_t PxInputData_tell(physx_PxInputData const* self__pod) {
        physx::PxInputData const* self_ = reinterpret_cast<physx::PxInputData const*>(self__pod);
        uint32_t return_val = self_->tell();
        return return_val;
    }

    void PxInputData_delete(physx_PxInputData* self__pod) {
        physx::PxInputData* self_ = reinterpret_cast<physx::PxInputData*>(self__pod);
        delete self_;
    }

    uint32_t PxOutputStream_write(physx_PxOutputStream* self__pod, void const* src, uint32_t count) {
        physx::PxOutputStream* self_ = reinterpret_cast<physx::PxOutputStream*>(self__pod);
        uint32_t return_val = self_->write(src, count);
        return return_val;
    }

    void PxOutputStream_delete(physx_PxOutputStream* self__pod) {
        physx::PxOutputStream* self_ = reinterpret_cast<physx::PxOutputStream*>(self__pod);
        delete self_;
    }

    physx_PxVec4 PxVec4_new() {
        PxVec4 return_val;
        physx_PxVec4 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec4 PxVec4_new_1(PxZERO anon_param0_pod) {
        auto anon_param0 = static_cast<physx::PxZERO>(anon_param0_pod);
        PxVec4 return_val(anon_param0);
        physx_PxVec4 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec4 PxVec4_new_2(float a) {
        PxVec4 return_val(a);
        physx_PxVec4 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec4 PxVec4_new_3(float nx, float ny, float nz, float nw) {
        PxVec4 return_val(nx, ny, nz, nw);
        physx_PxVec4 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec4 PxVec4_new_4(physx_PxVec3 const* v_pod, float nw) {
        physx::PxVec3 const& v = reinterpret_cast<physx::PxVec3 const&>(*v_pod);
        PxVec4 return_val(v, nw);
        physx_PxVec4 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec4 PxVec4_new_5(float const* v) {
        PxVec4 return_val(v);
        physx_PxVec4 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxVec4_isZero(physx_PxVec4 const* self__pod) {
        physx::PxVec4 const* self_ = reinterpret_cast<physx::PxVec4 const*>(self__pod);
        bool return_val = self_->isZero();
        return return_val;
    }

    bool PxVec4_isFinite(physx_PxVec4 const* self__pod) {
        physx::PxVec4 const* self_ = reinterpret_cast<physx::PxVec4 const*>(self__pod);
        bool return_val = self_->isFinite();
        return return_val;
    }

    bool PxVec4_isNormalized(physx_PxVec4 const* self__pod) {
        physx::PxVec4 const* self_ = reinterpret_cast<physx::PxVec4 const*>(self__pod);
        bool return_val = self_->isNormalized();
        return return_val;
    }

    float PxVec4_magnitudeSquared(physx_PxVec4 const* self__pod) {
        physx::PxVec4 const* self_ = reinterpret_cast<physx::PxVec4 const*>(self__pod);
        float return_val = self_->magnitudeSquared();
        return return_val;
    }

    float PxVec4_magnitude(physx_PxVec4 const* self__pod) {
        physx::PxVec4 const* self_ = reinterpret_cast<physx::PxVec4 const*>(self__pod);
        float return_val = self_->magnitude();
        return return_val;
    }

    float PxVec4_dot(physx_PxVec4 const* self__pod, physx_PxVec4 const* v_pod) {
        physx::PxVec4 const* self_ = reinterpret_cast<physx::PxVec4 const*>(self__pod);
        physx::PxVec4 const& v = reinterpret_cast<physx::PxVec4 const&>(*v_pod);
        float return_val = self_->dot(v);
        return return_val;
    }

    physx_PxVec4 PxVec4_getNormalized(physx_PxVec4 const* self__pod) {
        physx::PxVec4 const* self_ = reinterpret_cast<physx::PxVec4 const*>(self__pod);
        physx::PxVec4 return_val = self_->getNormalized();
        physx_PxVec4 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    float PxVec4_normalize(physx_PxVec4* self__pod) {
        physx::PxVec4* self_ = reinterpret_cast<physx::PxVec4*>(self__pod);
        float return_val = self_->normalize();
        return return_val;
    }

    physx_PxVec4 PxVec4_multiply(physx_PxVec4 const* self__pod, physx_PxVec4 const* a_pod) {
        physx::PxVec4 const* self_ = reinterpret_cast<physx::PxVec4 const*>(self__pod);
        physx::PxVec4 const& a = reinterpret_cast<physx::PxVec4 const&>(*a_pod);
        physx::PxVec4 return_val = self_->multiply(a);
        physx_PxVec4 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec4 PxVec4_minimum(physx_PxVec4 const* self__pod, physx_PxVec4 const* v_pod) {
        physx::PxVec4 const* self_ = reinterpret_cast<physx::PxVec4 const*>(self__pod);
        physx::PxVec4 const& v = reinterpret_cast<physx::PxVec4 const&>(*v_pod);
        physx::PxVec4 return_val = self_->minimum(v);
        physx_PxVec4 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec4 PxVec4_maximum(physx_PxVec4 const* self__pod, physx_PxVec4 const* v_pod) {
        physx::PxVec4 const* self_ = reinterpret_cast<physx::PxVec4 const*>(self__pod);
        physx::PxVec4 const& v = reinterpret_cast<physx::PxVec4 const&>(*v_pod);
        physx::PxVec4 return_val = self_->maximum(v);
        physx_PxVec4 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3 PxVec4_getXYZ(physx_PxVec4 const* self__pod) {
        physx::PxVec4 const* self_ = reinterpret_cast<physx::PxVec4 const*>(self__pod);
        physx::PxVec3 return_val = self_->getXYZ();
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxMat44 PxMat44_new() {
        PxMat44 return_val;
        physx_PxMat44 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxMat44 PxMat44_new_1(PxIDENTITY anon_param0_pod) {
        auto anon_param0 = static_cast<physx::PxIDENTITY>(anon_param0_pod);
        PxMat44 return_val(anon_param0);
        physx_PxMat44 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxMat44 PxMat44_new_2(PxZERO anon_param0_pod) {
        auto anon_param0 = static_cast<physx::PxZERO>(anon_param0_pod);
        PxMat44 return_val(anon_param0);
        physx_PxMat44 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxMat44 PxMat44_new_3(physx_PxVec4 const* col0_pod, physx_PxVec4 const* col1_pod, physx_PxVec4 const* col2_pod, physx_PxVec4 const* col3_pod) {
        physx::PxVec4 const& col0 = reinterpret_cast<physx::PxVec4 const&>(*col0_pod);
        physx::PxVec4 const& col1 = reinterpret_cast<physx::PxVec4 const&>(*col1_pod);
        physx::PxVec4 const& col2 = reinterpret_cast<physx::PxVec4 const&>(*col2_pod);
        physx::PxVec4 const& col3 = reinterpret_cast<physx::PxVec4 const&>(*col3_pod);
        PxMat44 return_val(col0, col1, col2, col3);
        physx_PxMat44 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxMat44 PxMat44_new_4(float r) {
        PxMat44 return_val(r);
        physx_PxMat44 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxMat44 PxMat44_new_5(physx_PxVec3 const* col0_pod, physx_PxVec3 const* col1_pod, physx_PxVec3 const* col2_pod, physx_PxVec3 const* col3_pod) {
        physx::PxVec3 const& col0 = reinterpret_cast<physx::PxVec3 const&>(*col0_pod);
        physx::PxVec3 const& col1 = reinterpret_cast<physx::PxVec3 const&>(*col1_pod);
        physx::PxVec3 const& col2 = reinterpret_cast<physx::PxVec3 const&>(*col2_pod);
        physx::PxVec3 const& col3 = reinterpret_cast<physx::PxVec3 const&>(*col3_pod);
        PxMat44 return_val(col0, col1, col2, col3);
        physx_PxMat44 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxMat44 PxMat44_new_6(float* values) {
        PxMat44 return_val(values);
        physx_PxMat44 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxMat44 PxMat44_new_7(physx_PxQuat const* q_pod) {
        physx::PxQuat const& q = reinterpret_cast<physx::PxQuat const&>(*q_pod);
        PxMat44 return_val(q);
        physx_PxMat44 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxMat44 PxMat44_new_8(physx_PxVec4 const* diagonal_pod) {
        physx::PxVec4 const& diagonal = reinterpret_cast<physx::PxVec4 const&>(*diagonal_pod);
        PxMat44 return_val(diagonal);
        physx_PxMat44 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxMat44 PxMat44_new_9(physx_PxMat33 const* axes_pod, physx_PxVec3 const* position_pod) {
        physx::PxMat33 const& axes = reinterpret_cast<physx::PxMat33 const&>(*axes_pod);
        physx::PxVec3 const& position = reinterpret_cast<physx::PxVec3 const&>(*position_pod);
        PxMat44 return_val(axes, position);
        physx_PxMat44 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxMat44 PxMat44_new_10(physx_PxTransform const* t_pod) {
        physx::PxTransform const& t = reinterpret_cast<physx::PxTransform const&>(*t_pod);
        PxMat44 return_val(t);
        physx_PxMat44 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxMat44 PxMat44_getTranspose(physx_PxMat44 const* self__pod) {
        physx::PxMat44 const* self_ = reinterpret_cast<physx::PxMat44 const*>(self__pod);
        physx::PxMat44 return_val = self_->getTranspose();
        physx_PxMat44 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec4 PxMat44_transform(physx_PxMat44 const* self__pod, physx_PxVec4 const* other_pod) {
        physx::PxMat44 const* self_ = reinterpret_cast<physx::PxMat44 const*>(self__pod);
        physx::PxVec4 const& other = reinterpret_cast<physx::PxVec4 const&>(*other_pod);
        physx::PxVec4 return_val = self_->transform(other);
        physx_PxVec4 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3 PxMat44_transform_1(physx_PxMat44 const* self__pod, physx_PxVec3 const* other_pod) {
        physx::PxMat44 const* self_ = reinterpret_cast<physx::PxMat44 const*>(self__pod);
        physx::PxVec3 const& other = reinterpret_cast<physx::PxVec3 const&>(*other_pod);
        physx::PxVec3 return_val = self_->transform(other);
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec4 PxMat44_rotate(physx_PxMat44 const* self__pod, physx_PxVec4 const* other_pod) {
        physx::PxMat44 const* self_ = reinterpret_cast<physx::PxMat44 const*>(self__pod);
        physx::PxVec4 const& other = reinterpret_cast<physx::PxVec4 const&>(*other_pod);
        physx::PxVec4 return_val = self_->rotate(other);
        physx_PxVec4 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3 PxMat44_rotate_1(physx_PxMat44 const* self__pod, physx_PxVec3 const* other_pod) {
        physx::PxMat44 const* self_ = reinterpret_cast<physx::PxMat44 const*>(self__pod);
        physx::PxVec3 const& other = reinterpret_cast<physx::PxVec3 const&>(*other_pod);
        physx::PxVec3 return_val = self_->rotate(other);
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3 PxMat44_getBasis(physx_PxMat44 const* self__pod, uint32_t num) {
        physx::PxMat44 const* self_ = reinterpret_cast<physx::PxMat44 const*>(self__pod);
        physx::PxVec3 return_val = self_->getBasis(num);
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3 PxMat44_getPosition(physx_PxMat44 const* self__pod) {
        physx::PxMat44 const* self_ = reinterpret_cast<physx::PxMat44 const*>(self__pod);
        physx::PxVec3 return_val = self_->getPosition();
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxMat44_setPosition(physx_PxMat44* self__pod, physx_PxVec3 const* position_pod) {
        physx::PxMat44* self_ = reinterpret_cast<physx::PxMat44*>(self__pod);
        physx::PxVec3 const& position = reinterpret_cast<physx::PxVec3 const&>(*position_pod);
        self_->setPosition(position);
    }

    float const* PxMat44_front(physx_PxMat44 const* self__pod) {
        physx::PxMat44 const* self_ = reinterpret_cast<physx::PxMat44 const*>(self__pod);
        float const* return_val = self_->front();
        return return_val;
    }

    void PxMat44_scale(physx_PxMat44* self__pod, physx_PxVec4 const* p_pod) {
        physx::PxMat44* self_ = reinterpret_cast<physx::PxMat44*>(self__pod);
        physx::PxVec4 const& p = reinterpret_cast<physx::PxVec4 const&>(*p_pod);
        self_->scale(p);
    }

    physx_PxMat44 PxMat44_inverseRT(physx_PxMat44 const* self__pod) {
        physx::PxMat44 const* self_ = reinterpret_cast<physx::PxMat44 const*>(self__pod);
        physx::PxMat44 return_val = self_->inverseRT();
        physx_PxMat44 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxMat44_isFinite(physx_PxMat44 const* self__pod) {
        physx::PxMat44 const* self_ = reinterpret_cast<physx::PxMat44 const*>(self__pod);
        bool return_val = self_->isFinite();
        return return_val;
    }

    physx_PxPlane PxPlane_new() {
        PxPlane return_val;
        physx_PxPlane return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxPlane PxPlane_new_1(float nx, float ny, float nz, float distance) {
        PxPlane return_val(nx, ny, nz, distance);
        physx_PxPlane return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxPlane PxPlane_new_2(physx_PxVec3 const* normal_pod, float distance) {
        physx::PxVec3 const& normal = reinterpret_cast<physx::PxVec3 const&>(*normal_pod);
        PxPlane return_val(normal, distance);
        physx_PxPlane return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxPlane PxPlane_new_3(physx_PxVec3 const* point_pod, physx_PxVec3 const* normal_pod) {
        physx::PxVec3 const& point = reinterpret_cast<physx::PxVec3 const&>(*point_pod);
        physx::PxVec3 const& normal = reinterpret_cast<physx::PxVec3 const&>(*normal_pod);
        PxPlane return_val(point, normal);
        physx_PxPlane return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxPlane PxPlane_new_4(physx_PxVec3 const* p0_pod, physx_PxVec3 const* p1_pod, physx_PxVec3 const* p2_pod) {
        physx::PxVec3 const& p0 = reinterpret_cast<physx::PxVec3 const&>(*p0_pod);
        physx::PxVec3 const& p1 = reinterpret_cast<physx::PxVec3 const&>(*p1_pod);
        physx::PxVec3 const& p2 = reinterpret_cast<physx::PxVec3 const&>(*p2_pod);
        PxPlane return_val(p0, p1, p2);
        physx_PxPlane return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    float PxPlane_distance(physx_PxPlane const* self__pod, physx_PxVec3 const* p_pod) {
        physx::PxPlane const* self_ = reinterpret_cast<physx::PxPlane const*>(self__pod);
        physx::PxVec3 const& p = reinterpret_cast<physx::PxVec3 const&>(*p_pod);
        float return_val = self_->distance(p);
        return return_val;
    }

    bool PxPlane_contains(physx_PxPlane const* self__pod, physx_PxVec3 const* p_pod) {
        physx::PxPlane const* self_ = reinterpret_cast<physx::PxPlane const*>(self__pod);
        physx::PxVec3 const& p = reinterpret_cast<physx::PxVec3 const&>(*p_pod);
        bool return_val = self_->contains(p);
        return return_val;
    }

    physx_PxVec3 PxPlane_project(physx_PxPlane const* self__pod, physx_PxVec3 const* p_pod) {
        physx::PxPlane const* self_ = reinterpret_cast<physx::PxPlane const*>(self__pod);
        physx::PxVec3 const& p = reinterpret_cast<physx::PxVec3 const&>(*p_pod);
        physx::PxVec3 return_val = self_->project(p);
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3 PxPlane_pointInPlane(physx_PxPlane const* self__pod) {
        physx::PxPlane const* self_ = reinterpret_cast<physx::PxPlane const*>(self__pod);
        physx::PxVec3 return_val = self_->pointInPlane();
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxPlane_normalize(physx_PxPlane* self__pod) {
        physx::PxPlane* self_ = reinterpret_cast<physx::PxPlane*>(self__pod);
        self_->normalize();
    }

    physx_PxPlane PxPlane_transform(physx_PxPlane const* self__pod, physx_PxTransform const* pose_pod) {
        physx::PxPlane const* self_ = reinterpret_cast<physx::PxPlane const*>(self__pod);
        physx::PxTransform const& pose = reinterpret_cast<physx::PxTransform const&>(*pose_pod);
        physx::PxPlane return_val = self_->transform(pose);
        physx_PxPlane return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxPlane PxPlane_inverseTransform(physx_PxPlane const* self__pod, physx_PxTransform const* pose_pod) {
        physx::PxPlane const* self_ = reinterpret_cast<physx::PxPlane const*>(self__pod);
        physx::PxTransform const& pose = reinterpret_cast<physx::PxTransform const&>(*pose_pod);
        physx::PxPlane return_val = self_->inverseTransform(pose);
        physx_PxPlane return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxQuat phys_PxShortestRotation(physx_PxVec3 const* from_pod, physx_PxVec3 const* target_pod) {
        physx::PxVec3 const& from = reinterpret_cast<physx::PxVec3 const&>(*from_pod);
        physx::PxVec3 const& target = reinterpret_cast<physx::PxVec3 const&>(*target_pod);
        physx::PxQuat return_val = PxShortestRotation(from, target);
        physx_PxQuat return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3 phys_PxDiagonalize(physx_PxMat33 const* m_pod, physx_PxQuat* axes_pod) {
        physx::PxMat33 const& m = reinterpret_cast<physx::PxMat33 const&>(*m_pod);
        physx::PxQuat& axes = reinterpret_cast<physx::PxQuat&>(*axes_pod);
        physx::PxVec3 return_val = PxDiagonalize(m, axes);
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxTransform phys_PxTransformFromSegment(physx_PxVec3 const* p0_pod, physx_PxVec3 const* p1_pod, float* halfHeight) {
        physx::PxVec3 const& p0 = reinterpret_cast<physx::PxVec3 const&>(*p0_pod);
        physx::PxVec3 const& p1 = reinterpret_cast<physx::PxVec3 const&>(*p1_pod);
        physx::PxTransform return_val = PxTransformFromSegment(p0, p1, halfHeight);
        physx_PxTransform return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxTransform phys_PxTransformFromPlaneEquation(physx_PxPlane const* plane_pod) {
        physx::PxPlane const& plane = reinterpret_cast<physx::PxPlane const&>(*plane_pod);
        physx::PxTransform return_val = PxTransformFromPlaneEquation(plane);
        physx_PxTransform return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxPlane phys_PxPlaneEquationFromTransform(physx_PxTransform const* pose_pod) {
        physx::PxTransform const& pose = reinterpret_cast<physx::PxTransform const&>(*pose_pod);
        physx::PxPlane return_val = PxPlaneEquationFromTransform(pose);
        physx_PxPlane return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxQuat phys_PxSlerp(float t, physx_PxQuat const* left_pod, physx_PxQuat const* right_pod) {
        physx::PxQuat const& left = reinterpret_cast<physx::PxQuat const&>(*left_pod);
        physx::PxQuat const& right = reinterpret_cast<physx::PxQuat const&>(*right_pod);
        physx::PxQuat return_val = PxSlerp(t, left, right);
        physx_PxQuat return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void phys_PxIntegrateTransform(physx_PxTransform const* curTrans_pod, physx_PxVec3 const* linvel_pod, physx_PxVec3 const* angvel_pod, float timeStep, physx_PxTransform* result_pod) {
        physx::PxTransform const& curTrans = reinterpret_cast<physx::PxTransform const&>(*curTrans_pod);
        physx::PxVec3 const& linvel = reinterpret_cast<physx::PxVec3 const&>(*linvel_pod);
        physx::PxVec3 const& angvel = reinterpret_cast<physx::PxVec3 const&>(*angvel_pod);
        physx::PxTransform& result = reinterpret_cast<physx::PxTransform&>(*result_pod);
        PxIntegrateTransform(curTrans, linvel, angvel, timeStep, result);
    }

    physx_PxQuat phys_PxExp(physx_PxVec3 const* v_pod) {
        physx::PxVec3 const& v = reinterpret_cast<physx::PxVec3 const&>(*v_pod);
        physx::PxQuat return_val = PxExp(v);
        physx_PxQuat return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3 phys_PxOptimizeBoundingBox(physx_PxMat33* basis_pod) {
        physx::PxMat33& basis = reinterpret_cast<physx::PxMat33&>(*basis_pod);
        physx::PxVec3 return_val = PxOptimizeBoundingBox(basis);
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3 phys_PxLog(physx_PxQuat const* q_pod) {
        physx::PxQuat const& q = reinterpret_cast<physx::PxQuat const&>(*q_pod);
        physx::PxVec3 return_val = PxLog(q);
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    uint32_t phys_PxLargestAxis(physx_PxVec3 const* v_pod) {
        physx::PxVec3 const& v = reinterpret_cast<physx::PxVec3 const&>(*v_pod);
        uint32_t return_val = PxLargestAxis(v);
        return return_val;
    }

    float phys_PxTanHalf(float sin, float cos) {
        float return_val = PxTanHalf(sin, cos);
        return return_val;
    }

    physx_PxVec3 phys_PxEllipseClamp(physx_PxVec3 const* point_pod, physx_PxVec3 const* radii_pod) {
        physx::PxVec3 const& point = reinterpret_cast<physx::PxVec3 const&>(*point_pod);
        physx::PxVec3 const& radii = reinterpret_cast<physx::PxVec3 const&>(*radii_pod);
        physx::PxVec3 return_val = PxEllipseClamp(point, radii);
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void phys_PxSeparateSwingTwist(physx_PxQuat const* q_pod, physx_PxQuat* swing_pod, physx_PxQuat* twist_pod) {
        physx::PxQuat const& q = reinterpret_cast<physx::PxQuat const&>(*q_pod);
        physx::PxQuat& swing = reinterpret_cast<physx::PxQuat&>(*swing_pod);
        physx::PxQuat& twist = reinterpret_cast<physx::PxQuat&>(*twist_pod);
        PxSeparateSwingTwist(q, swing, twist);
    }

    float phys_PxComputeAngle(physx_PxVec3 const* v0_pod, physx_PxVec3 const* v1_pod) {
        physx::PxVec3 const& v0 = reinterpret_cast<physx::PxVec3 const&>(*v0_pod);
        physx::PxVec3 const& v1 = reinterpret_cast<physx::PxVec3 const&>(*v1_pod);
        float return_val = PxComputeAngle(v0, v1);
        return return_val;
    }

    void phys_PxComputeBasisVectors(physx_PxVec3 const* dir_pod, physx_PxVec3* right_pod, physx_PxVec3* up_pod) {
        physx::PxVec3 const& dir = reinterpret_cast<physx::PxVec3 const&>(*dir_pod);
        physx::PxVec3& right = reinterpret_cast<physx::PxVec3&>(*right_pod);
        physx::PxVec3& up = reinterpret_cast<physx::PxVec3&>(*up_pod);
        PxComputeBasisVectors(dir, right, up);
    }

    void phys_PxComputeBasisVectors_1(physx_PxVec3 const* p0_pod, physx_PxVec3 const* p1_pod, physx_PxVec3* dir_pod, physx_PxVec3* right_pod, physx_PxVec3* up_pod) {
        physx::PxVec3 const& p0 = reinterpret_cast<physx::PxVec3 const&>(*p0_pod);
        physx::PxVec3 const& p1 = reinterpret_cast<physx::PxVec3 const&>(*p1_pod);
        physx::PxVec3& dir = reinterpret_cast<physx::PxVec3&>(*dir_pod);
        physx::PxVec3& right = reinterpret_cast<physx::PxVec3&>(*right_pod);
        physx::PxVec3& up = reinterpret_cast<physx::PxVec3&>(*up_pod);
        PxComputeBasisVectors(p0, p1, dir, right, up);
    }

    uint32_t phys_PxGetNextIndex3(uint32_t i) {
        uint32_t return_val = PxGetNextIndex3(i);
        return return_val;
    }

    void phys_computeBarycentric(physx_PxVec3 const* a_pod, physx_PxVec3 const* b_pod, physx_PxVec3 const* c_pod, physx_PxVec3 const* d_pod, physx_PxVec3 const* p_pod, physx_PxVec4* bary_pod) {
        physx::PxVec3 const& a = reinterpret_cast<physx::PxVec3 const&>(*a_pod);
        physx::PxVec3 const& b = reinterpret_cast<physx::PxVec3 const&>(*b_pod);
        physx::PxVec3 const& c = reinterpret_cast<physx::PxVec3 const&>(*c_pod);
        physx::PxVec3 const& d = reinterpret_cast<physx::PxVec3 const&>(*d_pod);
        physx::PxVec3 const& p = reinterpret_cast<physx::PxVec3 const&>(*p_pod);
        physx::PxVec4& bary = reinterpret_cast<physx::PxVec4&>(*bary_pod);
        computeBarycentric(a, b, c, d, p, bary);
    }

    void phys_computeBarycentric_1(physx_PxVec3 const* a_pod, physx_PxVec3 const* b_pod, physx_PxVec3 const* c_pod, physx_PxVec3 const* p_pod, physx_PxVec4* bary_pod) {
        physx::PxVec3 const& a = reinterpret_cast<physx::PxVec3 const&>(*a_pod);
        physx::PxVec3 const& b = reinterpret_cast<physx::PxVec3 const&>(*b_pod);
        physx::PxVec3 const& c = reinterpret_cast<physx::PxVec3 const&>(*c_pod);
        physx::PxVec3 const& p = reinterpret_cast<physx::PxVec3 const&>(*p_pod);
        physx::PxVec4& bary = reinterpret_cast<physx::PxVec4&>(*bary_pod);
        computeBarycentric(a, b, c, p, bary);
    }

    float Interpolation_PxLerp(float a, float b, float t) {
        float return_val = Interpolation::PxLerp(a, b, t);
        return return_val;
    }

    float Interpolation_PxBiLerp(float f00, float f10, float f01, float f11, float tx, float ty) {
        float return_val = Interpolation::PxBiLerp(f00, f10, f01, f11, tx, ty);
        return return_val;
    }

    float Interpolation_PxTriLerp(float f000, float f100, float f010, float f110, float f001, float f101, float f011, float f111, float tx, float ty, float tz) {
        float return_val = Interpolation::PxTriLerp(f000, f100, f010, f110, f001, f101, f011, f111, tx, ty, tz);
        return return_val;
    }

    uint32_t Interpolation_PxSDFIdx(uint32_t i, uint32_t j, uint32_t k, uint32_t nbX, uint32_t nbY) {
        uint32_t return_val = Interpolation::PxSDFIdx(i, j, k, nbX, nbY);
        return return_val;
    }

    float Interpolation_PxSDFSampleImpl(float const* sdf, physx_PxVec3 const* localPos_pod, physx_PxVec3 const* sdfBoxLower_pod, physx_PxVec3 const* sdfBoxHigher_pod, float sdfDx, float invSdfDx, uint32_t dimX, uint32_t dimY, uint32_t dimZ, float tolerance) {
        physx::PxVec3 const& localPos = reinterpret_cast<physx::PxVec3 const&>(*localPos_pod);
        physx::PxVec3 const& sdfBoxLower = reinterpret_cast<physx::PxVec3 const&>(*sdfBoxLower_pod);
        physx::PxVec3 const& sdfBoxHigher = reinterpret_cast<physx::PxVec3 const&>(*sdfBoxHigher_pod);
        float return_val = Interpolation::PxSDFSampleImpl(sdf, localPos, sdfBoxLower, sdfBoxHigher, sdfDx, invSdfDx, dimX, dimY, dimZ, tolerance);
        return return_val;
    }

    float phys_PxSdfSample(float const* sdf, physx_PxVec3 const* localPos_pod, physx_PxVec3 const* sdfBoxLower_pod, physx_PxVec3 const* sdfBoxHigher_pod, float sdfDx, float invSdfDx, uint32_t dimX, uint32_t dimY, uint32_t dimZ, physx_PxVec3* gradient_pod, float tolerance) {
        physx::PxVec3 const& localPos = reinterpret_cast<physx::PxVec3 const&>(*localPos_pod);
        physx::PxVec3 const& sdfBoxLower = reinterpret_cast<physx::PxVec3 const&>(*sdfBoxLower_pod);
        physx::PxVec3 const& sdfBoxHigher = reinterpret_cast<physx::PxVec3 const&>(*sdfBoxHigher_pod);
        physx::PxVec3& gradient = reinterpret_cast<physx::PxVec3&>(*gradient_pod);
        float return_val = PxSdfSample(sdf, localPos, sdfBoxLower, sdfBoxHigher, sdfDx, invSdfDx, dimX, dimY, dimZ, gradient, tolerance);
        return return_val;
    }

    physx_PxMutexImpl* PxMutexImpl_new_alloc() {
        auto return_val = new physx::PxMutexImpl();
        auto return_val_pod = reinterpret_cast<physx_PxMutexImpl*>(return_val);
        return return_val_pod;
    }

    void PxMutexImpl_delete(physx_PxMutexImpl* self__pod) {
        physx::PxMutexImpl* self_ = reinterpret_cast<physx::PxMutexImpl*>(self__pod);
        delete self_;
    }

    void PxMutexImpl_lock(physx_PxMutexImpl* self__pod) {
        physx::PxMutexImpl* self_ = reinterpret_cast<physx::PxMutexImpl*>(self__pod);
        self_->lock();
    }

    bool PxMutexImpl_trylock(physx_PxMutexImpl* self__pod) {
        physx::PxMutexImpl* self_ = reinterpret_cast<physx::PxMutexImpl*>(self__pod);
        bool return_val = self_->trylock();
        return return_val;
    }

    void PxMutexImpl_unlock(physx_PxMutexImpl* self__pod) {
        physx::PxMutexImpl* self_ = reinterpret_cast<physx::PxMutexImpl*>(self__pod);
        self_->unlock();
    }

    uint32_t PxMutexImpl_getSize() {
        uint32_t return_val = PxMutexImpl::getSize();
        return return_val;
    }

    physx_PxReadWriteLock* PxReadWriteLock_new_alloc() {
        auto return_val = new physx::PxReadWriteLock();
        auto return_val_pod = reinterpret_cast<physx_PxReadWriteLock*>(return_val);
        return return_val_pod;
    }

    void PxReadWriteLock_delete(physx_PxReadWriteLock* self__pod) {
        physx::PxReadWriteLock* self_ = reinterpret_cast<physx::PxReadWriteLock*>(self__pod);
        delete self_;
    }

    void PxReadWriteLock_lockReader(physx_PxReadWriteLock* self__pod, bool takeLock) {
        physx::PxReadWriteLock* self_ = reinterpret_cast<physx::PxReadWriteLock*>(self__pod);
        self_->lockReader(takeLock);
    }

    void PxReadWriteLock_lockWriter(physx_PxReadWriteLock* self__pod) {
        physx::PxReadWriteLock* self_ = reinterpret_cast<physx::PxReadWriteLock*>(self__pod);
        self_->lockWriter();
    }

    void PxReadWriteLock_unlockReader(physx_PxReadWriteLock* self__pod) {
        physx::PxReadWriteLock* self_ = reinterpret_cast<physx::PxReadWriteLock*>(self__pod);
        self_->unlockReader();
    }

    void PxReadWriteLock_unlockWriter(physx_PxReadWriteLock* self__pod) {
        physx::PxReadWriteLock* self_ = reinterpret_cast<physx::PxReadWriteLock*>(self__pod);
        self_->unlockWriter();
    }

    void* PxProfilerCallback_zoneStart(physx_PxProfilerCallback* self__pod, char const* eventName, bool detached, uint64_t contextId) {
        physx::PxProfilerCallback* self_ = reinterpret_cast<physx::PxProfilerCallback*>(self__pod);
        void* return_val = self_->zoneStart(eventName, detached, contextId);
        return return_val;
    }

    void PxProfilerCallback_zoneEnd(physx_PxProfilerCallback* self__pod, void* profilerData, char const* eventName, bool detached, uint64_t contextId) {
        physx::PxProfilerCallback* self_ = reinterpret_cast<physx::PxProfilerCallback*>(self__pod);
        self_->zoneEnd(profilerData, eventName, detached, contextId);
    }

    physx_PxProfileScoped* PxProfileScoped_new_alloc(physx_PxProfilerCallback* callback_pod, char const* eventName, bool detached, uint64_t contextId) {
        physx::PxProfilerCallback* callback = reinterpret_cast<physx::PxProfilerCallback*>(callback_pod);
        auto return_val = new physx::PxProfileScoped(callback, eventName, detached, contextId);
        auto return_val_pod = reinterpret_cast<physx_PxProfileScoped*>(return_val);
        return return_val_pod;
    }

    void PxProfileScoped_delete(physx_PxProfileScoped* self__pod) {
        physx::PxProfileScoped* self_ = reinterpret_cast<physx::PxProfileScoped*>(self__pod);
        delete self_;
    }

    physx_PxSListEntry PxSListEntry_new() {
        PxSListEntry return_val;
        physx_PxSListEntry return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxSListEntry* PxSListEntry_next(physx_PxSListEntry* self__pod) {
        physx::PxSListEntry* self_ = reinterpret_cast<physx::PxSListEntry*>(self__pod);
        physx::PxSListEntry* return_val = self_->next();
        auto return_val_pod = reinterpret_cast<physx_PxSListEntry*>(return_val);
        return return_val_pod;
    }

    physx_PxSListImpl* PxSListImpl_new_alloc() {
        auto return_val = new physx::PxSListImpl();
        auto return_val_pod = reinterpret_cast<physx_PxSListImpl*>(return_val);
        return return_val_pod;
    }

    void PxSListImpl_delete(physx_PxSListImpl* self__pod) {
        physx::PxSListImpl* self_ = reinterpret_cast<physx::PxSListImpl*>(self__pod);
        delete self_;
    }

    void PxSListImpl_push(physx_PxSListImpl* self__pod, physx_PxSListEntry* entry_pod) {
        physx::PxSListImpl* self_ = reinterpret_cast<physx::PxSListImpl*>(self__pod);
        physx::PxSListEntry* entry = reinterpret_cast<physx::PxSListEntry*>(entry_pod);
        self_->push(entry);
    }

    physx_PxSListEntry* PxSListImpl_pop(physx_PxSListImpl* self__pod) {
        physx::PxSListImpl* self_ = reinterpret_cast<physx::PxSListImpl*>(self__pod);
        physx::PxSListEntry* return_val = self_->pop();
        auto return_val_pod = reinterpret_cast<physx_PxSListEntry*>(return_val);
        return return_val_pod;
    }

    physx_PxSListEntry* PxSListImpl_flush(physx_PxSListImpl* self__pod) {
        physx::PxSListImpl* self_ = reinterpret_cast<physx::PxSListImpl*>(self__pod);
        physx::PxSListEntry* return_val = self_->flush();
        auto return_val_pod = reinterpret_cast<physx_PxSListEntry*>(return_val);
        return return_val_pod;
    }

    uint32_t PxSListImpl_getSize() {
        uint32_t return_val = PxSListImpl::getSize();
        return return_val;
    }

    physx_PxSyncImpl* PxSyncImpl_new_alloc() {
        auto return_val = new physx::PxSyncImpl();
        auto return_val_pod = reinterpret_cast<physx_PxSyncImpl*>(return_val);
        return return_val_pod;
    }

    void PxSyncImpl_delete(physx_PxSyncImpl* self__pod) {
        physx::PxSyncImpl* self_ = reinterpret_cast<physx::PxSyncImpl*>(self__pod);
        delete self_;
    }

    bool PxSyncImpl_wait(physx_PxSyncImpl* self__pod, uint32_t milliseconds) {
        physx::PxSyncImpl* self_ = reinterpret_cast<physx::PxSyncImpl*>(self__pod);
        bool return_val = self_->wait(milliseconds);
        return return_val;
    }

    void PxSyncImpl_set(physx_PxSyncImpl* self__pod) {
        physx::PxSyncImpl* self_ = reinterpret_cast<physx::PxSyncImpl*>(self__pod);
        self_->set();
    }

    void PxSyncImpl_reset(physx_PxSyncImpl* self__pod) {
        physx::PxSyncImpl* self_ = reinterpret_cast<physx::PxSyncImpl*>(self__pod);
        self_->reset();
    }

    uint32_t PxSyncImpl_getSize() {
        uint32_t return_val = PxSyncImpl::getSize();
        return return_val;
    }

    physx_PxRunnable* PxRunnable_new_alloc() {
        auto return_val = new physx::PxRunnable();
        auto return_val_pod = reinterpret_cast<physx_PxRunnable*>(return_val);
        return return_val_pod;
    }

    void PxRunnable_delete(physx_PxRunnable* self__pod) {
        physx::PxRunnable* self_ = reinterpret_cast<physx::PxRunnable*>(self__pod);
        delete self_;
    }

    void PxRunnable_execute(physx_PxRunnable* self__pod) {
        physx::PxRunnable* self_ = reinterpret_cast<physx::PxRunnable*>(self__pod);
        self_->execute();
    }

    uint32_t phys_PxTlsAlloc() {
        uint32_t return_val = PxTlsAlloc();
        return return_val;
    }

    void phys_PxTlsFree(uint32_t index) {
        PxTlsFree(index);
    }

    void* phys_PxTlsGet(uint32_t index) {
        void* return_val = PxTlsGet(index);
        return return_val;
    }

    size_t phys_PxTlsGetValue(uint32_t index) {
        size_t return_val = PxTlsGetValue(index);
        size_t return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    uint32_t phys_PxTlsSet(uint32_t index, void* value) {
        uint32_t return_val = PxTlsSet(index, value);
        return return_val;
    }

    uint32_t phys_PxTlsSetValue(uint32_t index, size_t value_pod) {
        size_t value;
        memcpy(&value, &value_pod, sizeof(value));
        uint32_t return_val = PxTlsSetValue(index, value);
        return return_val;
    }

    physx_PxCounterFrequencyToTensOfNanos PxCounterFrequencyToTensOfNanos_new(uint64_t inNum, uint64_t inDenom) {
        PxCounterFrequencyToTensOfNanos return_val(inNum, inDenom);
        physx_PxCounterFrequencyToTensOfNanos return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    uint64_t PxCounterFrequencyToTensOfNanos_toTensOfNanos(physx_PxCounterFrequencyToTensOfNanos const* self__pod, uint64_t inCounter) {
        physx::PxCounterFrequencyToTensOfNanos const* self_ = reinterpret_cast<physx::PxCounterFrequencyToTensOfNanos const*>(self__pod);
        uint64_t return_val = self_->toTensOfNanos(inCounter);
        return return_val;
    }

    physx_PxCounterFrequencyToTensOfNanos const* PxTime_getBootCounterFrequency() {
        physx::PxCounterFrequencyToTensOfNanos const& return_val = PxTime::getBootCounterFrequency();
        auto return_val_pod = reinterpret_cast<physx_PxCounterFrequencyToTensOfNanos const*>(&return_val);
        return return_val_pod;
    }

    physx_PxCounterFrequencyToTensOfNanos PxTime_getCounterFrequency() {
        physx::PxCounterFrequencyToTensOfNanos return_val = PxTime::getCounterFrequency();
        physx_PxCounterFrequencyToTensOfNanos return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    uint64_t PxTime_getCurrentCounterValue() {
        uint64_t return_val = PxTime::getCurrentCounterValue();
        return return_val;
    }

    uint64_t PxTime_getCurrentTimeInTensOfNanoSeconds() {
        uint64_t return_val = PxTime::getCurrentTimeInTensOfNanoSeconds();
        return return_val;
    }

    physx_PxTime PxTime_new() {
        PxTime return_val;
        physx_PxTime return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    double PxTime_getElapsedSeconds(physx_PxTime* self__pod) {
        physx::PxTime* self_ = reinterpret_cast<physx::PxTime*>(self__pod);
        double return_val = self_->getElapsedSeconds();
        return return_val;
    }

    double PxTime_peekElapsedSeconds(physx_PxTime* self__pod) {
        physx::PxTime* self_ = reinterpret_cast<physx::PxTime*>(self__pod);
        double return_val = self_->peekElapsedSeconds();
        return return_val;
    }

    double PxTime_getLastTime(physx_PxTime const* self__pod) {
        physx::PxTime const* self_ = reinterpret_cast<physx::PxTime const*>(self__pod);
        double return_val = self_->getLastTime();
        return return_val;
    }

    physx_PxVec2 PxVec2_new() {
        PxVec2 return_val;
        physx_PxVec2 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec2 PxVec2_new_1(PxZERO anon_param0_pod) {
        auto anon_param0 = static_cast<physx::PxZERO>(anon_param0_pod);
        PxVec2 return_val(anon_param0);
        physx_PxVec2 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec2 PxVec2_new_2(float a) {
        PxVec2 return_val(a);
        physx_PxVec2 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec2 PxVec2_new_3(float nx, float ny) {
        PxVec2 return_val(nx, ny);
        physx_PxVec2 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxVec2_isZero(physx_PxVec2 const* self__pod) {
        physx::PxVec2 const* self_ = reinterpret_cast<physx::PxVec2 const*>(self__pod);
        bool return_val = self_->isZero();
        return return_val;
    }

    bool PxVec2_isFinite(physx_PxVec2 const* self__pod) {
        physx::PxVec2 const* self_ = reinterpret_cast<physx::PxVec2 const*>(self__pod);
        bool return_val = self_->isFinite();
        return return_val;
    }

    bool PxVec2_isNormalized(physx_PxVec2 const* self__pod) {
        physx::PxVec2 const* self_ = reinterpret_cast<physx::PxVec2 const*>(self__pod);
        bool return_val = self_->isNormalized();
        return return_val;
    }

    float PxVec2_magnitudeSquared(physx_PxVec2 const* self__pod) {
        physx::PxVec2 const* self_ = reinterpret_cast<physx::PxVec2 const*>(self__pod);
        float return_val = self_->magnitudeSquared();
        return return_val;
    }

    float PxVec2_magnitude(physx_PxVec2 const* self__pod) {
        physx::PxVec2 const* self_ = reinterpret_cast<physx::PxVec2 const*>(self__pod);
        float return_val = self_->magnitude();
        return return_val;
    }

    float PxVec2_dot(physx_PxVec2 const* self__pod, physx_PxVec2 const* v_pod) {
        physx::PxVec2 const* self_ = reinterpret_cast<physx::PxVec2 const*>(self__pod);
        physx::PxVec2 const& v = reinterpret_cast<physx::PxVec2 const&>(*v_pod);
        float return_val = self_->dot(v);
        return return_val;
    }

    physx_PxVec2 PxVec2_getNormalized(physx_PxVec2 const* self__pod) {
        physx::PxVec2 const* self_ = reinterpret_cast<physx::PxVec2 const*>(self__pod);
        physx::PxVec2 return_val = self_->getNormalized();
        physx_PxVec2 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    float PxVec2_normalize(physx_PxVec2* self__pod) {
        physx::PxVec2* self_ = reinterpret_cast<physx::PxVec2*>(self__pod);
        float return_val = self_->normalize();
        return return_val;
    }

    physx_PxVec2 PxVec2_multiply(physx_PxVec2 const* self__pod, physx_PxVec2 const* a_pod) {
        physx::PxVec2 const* self_ = reinterpret_cast<physx::PxVec2 const*>(self__pod);
        physx::PxVec2 const& a = reinterpret_cast<physx::PxVec2 const&>(*a_pod);
        physx::PxVec2 return_val = self_->multiply(a);
        physx_PxVec2 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec2 PxVec2_minimum(physx_PxVec2 const* self__pod, physx_PxVec2 const* v_pod) {
        physx::PxVec2 const* self_ = reinterpret_cast<physx::PxVec2 const*>(self__pod);
        physx::PxVec2 const& v = reinterpret_cast<physx::PxVec2 const&>(*v_pod);
        physx::PxVec2 return_val = self_->minimum(v);
        physx_PxVec2 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    float PxVec2_minElement(physx_PxVec2 const* self__pod) {
        physx::PxVec2 const* self_ = reinterpret_cast<physx::PxVec2 const*>(self__pod);
        float return_val = self_->minElement();
        return return_val;
    }

    physx_PxVec2 PxVec2_maximum(physx_PxVec2 const* self__pod, physx_PxVec2 const* v_pod) {
        physx::PxVec2 const* self_ = reinterpret_cast<physx::PxVec2 const*>(self__pod);
        physx::PxVec2 const& v = reinterpret_cast<physx::PxVec2 const&>(*v_pod);
        physx::PxVec2 return_val = self_->maximum(v);
        physx_PxVec2 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    float PxVec2_maxElement(physx_PxVec2 const* self__pod) {
        physx::PxVec2 const* self_ = reinterpret_cast<physx::PxVec2 const*>(self__pod);
        float return_val = self_->maxElement();
        return return_val;
    }

    physx_PxStridedData PxStridedData_new() {
        PxStridedData return_val;
        physx_PxStridedData return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxBoundedData PxBoundedData_new() {
        PxBoundedData return_val;
        physx_PxBoundedData return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxDebugPoint PxDebugPoint_new(physx_PxVec3 const* p_pod, uint32_t const* c_pod) {
        physx::PxVec3 const& p = reinterpret_cast<physx::PxVec3 const&>(*p_pod);
        uint32_t const& c = *c_pod;
        PxDebugPoint return_val(p, c);
        physx_PxDebugPoint return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxDebugLine PxDebugLine_new(physx_PxVec3 const* p0_pod, physx_PxVec3 const* p1_pod, uint32_t const* c_pod) {
        physx::PxVec3 const& p0 = reinterpret_cast<physx::PxVec3 const&>(*p0_pod);
        physx::PxVec3 const& p1 = reinterpret_cast<physx::PxVec3 const&>(*p1_pod);
        uint32_t const& c = *c_pod;
        PxDebugLine return_val(p0, p1, c);
        physx_PxDebugLine return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxDebugTriangle PxDebugTriangle_new(physx_PxVec3 const* p0_pod, physx_PxVec3 const* p1_pod, physx_PxVec3 const* p2_pod, uint32_t const* c_pod) {
        physx::PxVec3 const& p0 = reinterpret_cast<physx::PxVec3 const&>(*p0_pod);
        physx::PxVec3 const& p1 = reinterpret_cast<physx::PxVec3 const&>(*p1_pod);
        physx::PxVec3 const& p2 = reinterpret_cast<physx::PxVec3 const&>(*p2_pod);
        uint32_t const& c = *c_pod;
        PxDebugTriangle return_val(p0, p1, p2, c);
        physx_PxDebugTriangle return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxDebugText PxDebugText_new() {
        PxDebugText return_val;
        physx_PxDebugText return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxDebugText PxDebugText_new_1(physx_PxVec3 const* pos_pod, float const* sz_pod, uint32_t const* clr_pod, char const* str) {
        physx::PxVec3 const& pos = reinterpret_cast<physx::PxVec3 const&>(*pos_pod);
        float const& sz = *sz_pod;
        uint32_t const& clr = *clr_pod;
        PxDebugText return_val(pos, sz, clr, str);
        physx_PxDebugText return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxRenderBuffer_delete(physx_PxRenderBuffer* self__pod) {
        physx::PxRenderBuffer* self_ = reinterpret_cast<physx::PxRenderBuffer*>(self__pod);
        delete self_;
    }

    uint32_t PxRenderBuffer_getNbPoints(physx_PxRenderBuffer const* self__pod) {
        physx::PxRenderBuffer const* self_ = reinterpret_cast<physx::PxRenderBuffer const*>(self__pod);
        uint32_t return_val = self_->getNbPoints();
        return return_val;
    }

    physx_PxDebugPoint const* PxRenderBuffer_getPoints(physx_PxRenderBuffer const* self__pod) {
        physx::PxRenderBuffer const* self_ = reinterpret_cast<physx::PxRenderBuffer const*>(self__pod);
        physx::PxDebugPoint const* return_val = self_->getPoints();
        auto return_val_pod = reinterpret_cast<physx_PxDebugPoint const*>(return_val);
        return return_val_pod;
    }

    void PxRenderBuffer_addPoint(physx_PxRenderBuffer* self__pod, physx_PxDebugPoint const* point_pod) {
        physx::PxRenderBuffer* self_ = reinterpret_cast<physx::PxRenderBuffer*>(self__pod);
        physx::PxDebugPoint const& point = reinterpret_cast<physx::PxDebugPoint const&>(*point_pod);
        self_->addPoint(point);
    }

    uint32_t PxRenderBuffer_getNbLines(physx_PxRenderBuffer const* self__pod) {
        physx::PxRenderBuffer const* self_ = reinterpret_cast<physx::PxRenderBuffer const*>(self__pod);
        uint32_t return_val = self_->getNbLines();
        return return_val;
    }

    physx_PxDebugLine const* PxRenderBuffer_getLines(physx_PxRenderBuffer const* self__pod) {
        physx::PxRenderBuffer const* self_ = reinterpret_cast<physx::PxRenderBuffer const*>(self__pod);
        physx::PxDebugLine const* return_val = self_->getLines();
        auto return_val_pod = reinterpret_cast<physx_PxDebugLine const*>(return_val);
        return return_val_pod;
    }

    void PxRenderBuffer_addLine(physx_PxRenderBuffer* self__pod, physx_PxDebugLine const* line_pod) {
        physx::PxRenderBuffer* self_ = reinterpret_cast<physx::PxRenderBuffer*>(self__pod);
        physx::PxDebugLine const& line = reinterpret_cast<physx::PxDebugLine const&>(*line_pod);
        self_->addLine(line);
    }

    physx_PxDebugLine* PxRenderBuffer_reserveLines(physx_PxRenderBuffer* self__pod, uint32_t nbLines) {
        physx::PxRenderBuffer* self_ = reinterpret_cast<physx::PxRenderBuffer*>(self__pod);
        physx::PxDebugLine* return_val = self_->reserveLines(nbLines);
        auto return_val_pod = reinterpret_cast<physx_PxDebugLine*>(return_val);
        return return_val_pod;
    }

    physx_PxDebugPoint* PxRenderBuffer_reservePoints(physx_PxRenderBuffer* self__pod, uint32_t nbLines) {
        physx::PxRenderBuffer* self_ = reinterpret_cast<physx::PxRenderBuffer*>(self__pod);
        physx::PxDebugPoint* return_val = self_->reservePoints(nbLines);
        auto return_val_pod = reinterpret_cast<physx_PxDebugPoint*>(return_val);
        return return_val_pod;
    }

    uint32_t PxRenderBuffer_getNbTriangles(physx_PxRenderBuffer const* self__pod) {
        physx::PxRenderBuffer const* self_ = reinterpret_cast<physx::PxRenderBuffer const*>(self__pod);
        uint32_t return_val = self_->getNbTriangles();
        return return_val;
    }

    physx_PxDebugTriangle const* PxRenderBuffer_getTriangles(physx_PxRenderBuffer const* self__pod) {
        physx::PxRenderBuffer const* self_ = reinterpret_cast<physx::PxRenderBuffer const*>(self__pod);
        physx::PxDebugTriangle const* return_val = self_->getTriangles();
        auto return_val_pod = reinterpret_cast<physx_PxDebugTriangle const*>(return_val);
        return return_val_pod;
    }

    void PxRenderBuffer_addTriangle(physx_PxRenderBuffer* self__pod, physx_PxDebugTriangle const* triangle_pod) {
        physx::PxRenderBuffer* self_ = reinterpret_cast<physx::PxRenderBuffer*>(self__pod);
        physx::PxDebugTriangle const& triangle = reinterpret_cast<physx::PxDebugTriangle const&>(*triangle_pod);
        self_->addTriangle(triangle);
    }

    void PxRenderBuffer_append(physx_PxRenderBuffer* self__pod, physx_PxRenderBuffer const* other_pod) {
        physx::PxRenderBuffer* self_ = reinterpret_cast<physx::PxRenderBuffer*>(self__pod);
        physx::PxRenderBuffer const& other = reinterpret_cast<physx::PxRenderBuffer const&>(*other_pod);
        self_->append(other);
    }

    void PxRenderBuffer_clear(physx_PxRenderBuffer* self__pod) {
        physx::PxRenderBuffer* self_ = reinterpret_cast<physx::PxRenderBuffer*>(self__pod);
        self_->clear();
    }

    void PxRenderBuffer_shift(physx_PxRenderBuffer* self__pod, physx_PxVec3 const* delta_pod) {
        physx::PxRenderBuffer* self_ = reinterpret_cast<physx::PxRenderBuffer*>(self__pod);
        physx::PxVec3 const& delta = reinterpret_cast<physx::PxVec3 const&>(*delta_pod);
        self_->shift(delta);
    }

    bool PxRenderBuffer_empty(physx_PxRenderBuffer const* self__pod) {
        physx::PxRenderBuffer const* self_ = reinterpret_cast<physx::PxRenderBuffer const*>(self__pod);
        bool return_val = self_->empty();
        return return_val;
    }

    void PxProcessPxBaseCallback_delete(physx_PxProcessPxBaseCallback* self__pod) {
        physx::PxProcessPxBaseCallback* self_ = reinterpret_cast<physx::PxProcessPxBaseCallback*>(self__pod);
        delete self_;
    }

    void PxProcessPxBaseCallback_process(physx_PxProcessPxBaseCallback* self__pod, physx_PxBase* anon_param0_pod) {
        physx::PxProcessPxBaseCallback* self_ = reinterpret_cast<physx::PxProcessPxBaseCallback*>(self__pod);
        physx::PxBase& anon_param0 = reinterpret_cast<physx::PxBase&>(*anon_param0_pod);
        self_->process(anon_param0);
    }

    void PxSerializationContext_registerReference(physx_PxSerializationContext* self__pod, physx_PxBase* base_pod, uint32_t kind, size_t reference_pod) {
        physx::PxSerializationContext* self_ = reinterpret_cast<physx::PxSerializationContext*>(self__pod);
        physx::PxBase& base = reinterpret_cast<physx::PxBase&>(*base_pod);
        size_t reference;
        memcpy(&reference, &reference_pod, sizeof(reference));
        self_->registerReference(base, kind, reference);
    }

    physx_PxCollection const* PxSerializationContext_getCollection(physx_PxSerializationContext const* self__pod) {
        physx::PxSerializationContext const* self_ = reinterpret_cast<physx::PxSerializationContext const*>(self__pod);
        physx::PxCollection const& return_val = self_->getCollection();
        auto return_val_pod = reinterpret_cast<physx_PxCollection const*>(&return_val);
        return return_val_pod;
    }

    void PxSerializationContext_writeData(physx_PxSerializationContext* self__pod, void const* data, uint32_t size) {
        physx::PxSerializationContext* self_ = reinterpret_cast<physx::PxSerializationContext*>(self__pod);
        self_->writeData(data, size);
    }

    void PxSerializationContext_alignData(physx_PxSerializationContext* self__pod, uint32_t alignment) {
        physx::PxSerializationContext* self_ = reinterpret_cast<physx::PxSerializationContext*>(self__pod);
        self_->alignData(alignment);
    }

    void PxSerializationContext_writeName(physx_PxSerializationContext* self__pod, char const* name) {
        physx::PxSerializationContext* self_ = reinterpret_cast<physx::PxSerializationContext*>(self__pod);
        self_->writeName(name);
    }

    physx_PxBase* PxDeserializationContext_resolveReference(physx_PxDeserializationContext const* self__pod, uint32_t kind, size_t reference_pod) {
        physx::PxDeserializationContext const* self_ = reinterpret_cast<physx::PxDeserializationContext const*>(self__pod);
        size_t reference;
        memcpy(&reference, &reference_pod, sizeof(reference));
        physx::PxBase* return_val = self_->resolveReference(kind, reference);
        auto return_val_pod = reinterpret_cast<physx_PxBase*>(return_val);
        return return_val_pod;
    }

    void PxDeserializationContext_readName(physx_PxDeserializationContext* self__pod, char const** name_pod) {
        physx::PxDeserializationContext* self_ = reinterpret_cast<physx::PxDeserializationContext*>(self__pod);
        char const*& name = reinterpret_cast<char const*&>(*name_pod);
        self_->readName(name);
    }

    void PxDeserializationContext_alignExtraData(physx_PxDeserializationContext* self__pod, uint32_t alignment) {
        physx::PxDeserializationContext* self_ = reinterpret_cast<physx::PxDeserializationContext*>(self__pod);
        self_->alignExtraData(alignment);
    }

    void PxSerializationRegistry_registerSerializer(physx_PxSerializationRegistry* self__pod, uint16_t type, physx_PxSerializer* serializer_pod) {
        physx::PxSerializationRegistry* self_ = reinterpret_cast<physx::PxSerializationRegistry*>(self__pod);
        physx::PxSerializer& serializer = reinterpret_cast<physx::PxSerializer&>(*serializer_pod);
        self_->registerSerializer(type, serializer);
    }

    physx_PxSerializer* PxSerializationRegistry_unregisterSerializer(physx_PxSerializationRegistry* self__pod, uint16_t type) {
        physx::PxSerializationRegistry* self_ = reinterpret_cast<physx::PxSerializationRegistry*>(self__pod);
        physx::PxSerializer* return_val = self_->unregisterSerializer(type);
        auto return_val_pod = reinterpret_cast<physx_PxSerializer*>(return_val);
        return return_val_pod;
    }

    physx_PxSerializer const* PxSerializationRegistry_getSerializer(physx_PxSerializationRegistry const* self__pod, uint16_t type) {
        physx::PxSerializationRegistry const* self_ = reinterpret_cast<physx::PxSerializationRegistry const*>(self__pod);
        physx::PxSerializer const* return_val = self_->getSerializer(type);
        auto return_val_pod = reinterpret_cast<physx_PxSerializer const*>(return_val);
        return return_val_pod;
    }

    void PxSerializationRegistry_registerRepXSerializer(physx_PxSerializationRegistry* self__pod, uint16_t type, physx_PxRepXSerializer* serializer_pod) {
        physx::PxSerializationRegistry* self_ = reinterpret_cast<physx::PxSerializationRegistry*>(self__pod);
        physx::PxRepXSerializer& serializer = reinterpret_cast<physx::PxRepXSerializer&>(*serializer_pod);
        self_->registerRepXSerializer(type, serializer);
    }

    physx_PxRepXSerializer* PxSerializationRegistry_unregisterRepXSerializer(physx_PxSerializationRegistry* self__pod, uint16_t type) {
        physx::PxSerializationRegistry* self_ = reinterpret_cast<physx::PxSerializationRegistry*>(self__pod);
        physx::PxRepXSerializer* return_val = self_->unregisterRepXSerializer(type);
        auto return_val_pod = reinterpret_cast<physx_PxRepXSerializer*>(return_val);
        return return_val_pod;
    }

    physx_PxRepXSerializer* PxSerializationRegistry_getRepXSerializer(physx_PxSerializationRegistry const* self__pod, char const* typeName) {
        physx::PxSerializationRegistry const* self_ = reinterpret_cast<physx::PxSerializationRegistry const*>(self__pod);
        physx::PxRepXSerializer* return_val = self_->getRepXSerializer(typeName);
        auto return_val_pod = reinterpret_cast<physx_PxRepXSerializer*>(return_val);
        return return_val_pod;
    }

    void PxSerializationRegistry_release(physx_PxSerializationRegistry* self__pod) {
        physx::PxSerializationRegistry* self_ = reinterpret_cast<physx::PxSerializationRegistry*>(self__pod);
        self_->release();
    }

    void PxCollection_add(physx_PxCollection* self__pod, physx_PxBase* object_pod, uint64_t id) {
        physx::PxCollection* self_ = reinterpret_cast<physx::PxCollection*>(self__pod);
        physx::PxBase& object = reinterpret_cast<physx::PxBase&>(*object_pod);
        self_->add(object, id);
    }

    void PxCollection_remove(physx_PxCollection* self__pod, physx_PxBase* object_pod) {
        physx::PxCollection* self_ = reinterpret_cast<physx::PxCollection*>(self__pod);
        physx::PxBase& object = reinterpret_cast<physx::PxBase&>(*object_pod);
        self_->remove(object);
    }

    bool PxCollection_contains(physx_PxCollection const* self__pod, physx_PxBase* object_pod) {
        physx::PxCollection const* self_ = reinterpret_cast<physx::PxCollection const*>(self__pod);
        physx::PxBase& object = reinterpret_cast<physx::PxBase&>(*object_pod);
        bool return_val = self_->contains(object);
        return return_val;
    }

    void PxCollection_addId(physx_PxCollection* self__pod, physx_PxBase* object_pod, uint64_t id) {
        physx::PxCollection* self_ = reinterpret_cast<physx::PxCollection*>(self__pod);
        physx::PxBase& object = reinterpret_cast<physx::PxBase&>(*object_pod);
        self_->addId(object, id);
    }

    void PxCollection_removeId(physx_PxCollection* self__pod, uint64_t id) {
        physx::PxCollection* self_ = reinterpret_cast<physx::PxCollection*>(self__pod);
        self_->removeId(id);
    }

    void PxCollection_add_1(physx_PxCollection* self__pod, physx_PxCollection* collection_pod) {
        physx::PxCollection* self_ = reinterpret_cast<physx::PxCollection*>(self__pod);
        physx::PxCollection& collection = reinterpret_cast<physx::PxCollection&>(*collection_pod);
        self_->add(collection);
    }

    void PxCollection_remove_1(physx_PxCollection* self__pod, physx_PxCollection* collection_pod) {
        physx::PxCollection* self_ = reinterpret_cast<physx::PxCollection*>(self__pod);
        physx::PxCollection& collection = reinterpret_cast<physx::PxCollection&>(*collection_pod);
        self_->remove(collection);
    }

    uint32_t PxCollection_getNbObjects(physx_PxCollection const* self__pod) {
        physx::PxCollection const* self_ = reinterpret_cast<physx::PxCollection const*>(self__pod);
        uint32_t return_val = self_->getNbObjects();
        return return_val;
    }

    physx_PxBase* PxCollection_getObject(physx_PxCollection const* self__pod, uint32_t index) {
        physx::PxCollection const* self_ = reinterpret_cast<physx::PxCollection const*>(self__pod);
        physx::PxBase& return_val = self_->getObject(index);
        auto return_val_pod = reinterpret_cast<physx_PxBase*>(&return_val);
        return return_val_pod;
    }

    uint32_t PxCollection_getObjects(physx_PxCollection const* self__pod, physx_PxBase** userBuffer_pod, uint32_t bufferSize, uint32_t startIndex) {
        physx::PxCollection const* self_ = reinterpret_cast<physx::PxCollection const*>(self__pod);
        physx::PxBase** userBuffer = reinterpret_cast<physx::PxBase**>(userBuffer_pod);
        uint32_t return_val = self_->getObjects(userBuffer, bufferSize, startIndex);
        return return_val;
    }

    physx_PxBase* PxCollection_find(physx_PxCollection const* self__pod, uint64_t id) {
        physx::PxCollection const* self_ = reinterpret_cast<physx::PxCollection const*>(self__pod);
        physx::PxBase* return_val = self_->find(id);
        auto return_val_pod = reinterpret_cast<physx_PxBase*>(return_val);
        return return_val_pod;
    }

    uint32_t PxCollection_getNbIds(physx_PxCollection const* self__pod) {
        physx::PxCollection const* self_ = reinterpret_cast<physx::PxCollection const*>(self__pod);
        uint32_t return_val = self_->getNbIds();
        return return_val;
    }

    uint32_t PxCollection_getIds(physx_PxCollection const* self__pod, uint64_t* userBuffer, uint32_t bufferSize, uint32_t startIndex) {
        physx::PxCollection const* self_ = reinterpret_cast<physx::PxCollection const*>(self__pod);
        uint32_t return_val = self_->getIds(userBuffer, bufferSize, startIndex);
        return return_val;
    }

    uint64_t PxCollection_getId(physx_PxCollection const* self__pod, physx_PxBase const* object_pod) {
        physx::PxCollection const* self_ = reinterpret_cast<physx::PxCollection const*>(self__pod);
        physx::PxBase const& object = reinterpret_cast<physx::PxBase const&>(*object_pod);
        uint64_t return_val = self_->getId(object);
        return return_val;
    }

    void PxCollection_release(physx_PxCollection* self__pod) {
        physx::PxCollection* self_ = reinterpret_cast<physx::PxCollection*>(self__pod);
        self_->release();
    }

    physx_PxCollection* phys_PxCreateCollection() {
        physx::PxCollection* return_val = PxCreateCollection();
        auto return_val_pod = reinterpret_cast<physx_PxCollection*>(return_val);
        return return_val_pod;
    }

    void PxBase_release(physx_PxBase* self__pod) {
        physx::PxBase* self_ = reinterpret_cast<physx::PxBase*>(self__pod);
        self_->release();
    }

    char const* PxBase_getConcreteTypeName(physx_PxBase const* self__pod) {
        physx::PxBase const* self_ = reinterpret_cast<physx::PxBase const*>(self__pod);
        char const* return_val = self_->getConcreteTypeName();
        return return_val;
    }

    uint16_t PxBase_getConcreteType(physx_PxBase const* self__pod) {
        physx::PxBase const* self_ = reinterpret_cast<physx::PxBase const*>(self__pod);
        uint16_t return_val = self_->getConcreteType();
        return return_val;
    }

    void PxBase_setBaseFlag(physx_PxBase* self__pod, PxBaseFlag flag_pod, bool value) {
        physx::PxBase* self_ = reinterpret_cast<physx::PxBase*>(self__pod);
        auto flag = static_cast<physx::PxBaseFlag::Enum>(flag_pod);
        self_->setBaseFlag(flag, value);
    }

    void PxBase_setBaseFlags(physx_PxBase* self__pod, PxBaseFlags inFlags_pod) {
        physx::PxBase* self_ = reinterpret_cast<physx::PxBase*>(self__pod);
        auto inFlags = physx::PxBaseFlags(inFlags_pod);
        self_->setBaseFlags(inFlags);
    }

    PxBaseFlags PxBase_getBaseFlags(physx_PxBase const* self__pod) {
        physx::PxBase const* self_ = reinterpret_cast<physx::PxBase const*>(self__pod);
        physx::PxBaseFlags return_val = self_->getBaseFlags();
        PxBaseFlags return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxBase_isReleasable(physx_PxBase const* self__pod) {
        physx::PxBase const* self_ = reinterpret_cast<physx::PxBase const*>(self__pod);
        bool return_val = self_->isReleasable();
        return return_val;
    }

    void PxRefCounted_release(physx_PxRefCounted* self__pod) {
        physx::PxRefCounted* self_ = reinterpret_cast<physx::PxRefCounted*>(self__pod);
        self_->release();
    }

    uint32_t PxRefCounted_getReferenceCount(physx_PxRefCounted const* self__pod) {
        physx::PxRefCounted const* self_ = reinterpret_cast<physx::PxRefCounted const*>(self__pod);
        uint32_t return_val = self_->getReferenceCount();
        return return_val;
    }

    void PxRefCounted_acquireReference(physx_PxRefCounted* self__pod) {
        physx::PxRefCounted* self_ = reinterpret_cast<physx::PxRefCounted*>(self__pod);
        self_->acquireReference();
    }

    physx_PxTolerancesScale PxTolerancesScale_new(float defaultLength, float defaultSpeed) {
        PxTolerancesScale return_val(defaultLength, defaultSpeed);
        physx_PxTolerancesScale return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxTolerancesScale_isValid(physx_PxTolerancesScale const* self__pod) {
        physx::PxTolerancesScale const* self_ = reinterpret_cast<physx::PxTolerancesScale const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    char const* PxStringTable_allocateStr(physx_PxStringTable* self__pod, char const* inSrc) {
        physx::PxStringTable* self_ = reinterpret_cast<physx::PxStringTable*>(self__pod);
        char const* return_val = self_->allocateStr(inSrc);
        return return_val;
    }

    void PxStringTable_release(physx_PxStringTable* self__pod) {
        physx::PxStringTable* self_ = reinterpret_cast<physx::PxStringTable*>(self__pod);
        self_->release();
    }

    char const* PxSerializer_getConcreteTypeName(physx_PxSerializer const* self__pod) {
        physx::PxSerializer const* self_ = reinterpret_cast<physx::PxSerializer const*>(self__pod);
        char const* return_val = self_->getConcreteTypeName();
        return return_val;
    }

    void PxSerializer_requiresObjects(physx_PxSerializer const* self__pod, physx_PxBase* anon_param0_pod, physx_PxProcessPxBaseCallback* anon_param1_pod) {
        physx::PxSerializer const* self_ = reinterpret_cast<physx::PxSerializer const*>(self__pod);
        physx::PxBase& anon_param0 = reinterpret_cast<physx::PxBase&>(*anon_param0_pod);
        physx::PxProcessPxBaseCallback& anon_param1 = reinterpret_cast<physx::PxProcessPxBaseCallback&>(*anon_param1_pod);
        self_->requiresObjects(anon_param0, anon_param1);
    }

    bool PxSerializer_isSubordinate(physx_PxSerializer const* self__pod) {
        physx::PxSerializer const* self_ = reinterpret_cast<physx::PxSerializer const*>(self__pod);
        bool return_val = self_->isSubordinate();
        return return_val;
    }

    void PxSerializer_exportExtraData(physx_PxSerializer const* self__pod, physx_PxBase* anon_param0_pod, physx_PxSerializationContext* anon_param1_pod) {
        physx::PxSerializer const* self_ = reinterpret_cast<physx::PxSerializer const*>(self__pod);
        physx::PxBase& anon_param0 = reinterpret_cast<physx::PxBase&>(*anon_param0_pod);
        physx::PxSerializationContext& anon_param1 = reinterpret_cast<physx::PxSerializationContext&>(*anon_param1_pod);
        self_->exportExtraData(anon_param0, anon_param1);
    }

    void PxSerializer_exportData(physx_PxSerializer const* self__pod, physx_PxBase* anon_param0_pod, physx_PxSerializationContext* anon_param1_pod) {
        physx::PxSerializer const* self_ = reinterpret_cast<physx::PxSerializer const*>(self__pod);
        physx::PxBase& anon_param0 = reinterpret_cast<physx::PxBase&>(*anon_param0_pod);
        physx::PxSerializationContext& anon_param1 = reinterpret_cast<physx::PxSerializationContext&>(*anon_param1_pod);
        self_->exportData(anon_param0, anon_param1);
    }

    void PxSerializer_registerReferences(physx_PxSerializer const* self__pod, physx_PxBase* obj_pod, physx_PxSerializationContext* s_pod) {
        physx::PxSerializer const* self_ = reinterpret_cast<physx::PxSerializer const*>(self__pod);
        physx::PxBase& obj = reinterpret_cast<physx::PxBase&>(*obj_pod);
        physx::PxSerializationContext& s = reinterpret_cast<physx::PxSerializationContext&>(*s_pod);
        self_->registerReferences(obj, s);
    }

    size_t PxSerializer_getClassSize(physx_PxSerializer const* self__pod) {
        physx::PxSerializer const* self_ = reinterpret_cast<physx::PxSerializer const*>(self__pod);
        size_t return_val = self_->getClassSize();
        size_t return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxBase* PxSerializer_createObject(physx_PxSerializer const* self__pod, uint8_t** address_pod, physx_PxDeserializationContext* context_pod) {
        physx::PxSerializer const* self_ = reinterpret_cast<physx::PxSerializer const*>(self__pod);
        uint8_t*& address = reinterpret_cast<uint8_t*&>(*address_pod);
        physx::PxDeserializationContext& context = reinterpret_cast<physx::PxDeserializationContext&>(*context_pod);
        physx::PxBase* return_val = self_->createObject(address, context);
        auto return_val_pod = reinterpret_cast<physx_PxBase*>(return_val);
        return return_val_pod;
    }

    void PxSerializer_delete(physx_PxSerializer* self__pod) {
        physx::PxSerializer* self_ = reinterpret_cast<physx::PxSerializer*>(self__pod);
        delete self_;
    }

    physx_PxBase* PxInsertionCallback_buildObjectFromData(physx_PxInsertionCallback* self__pod, PxConcreteType type_pod, void* data) {
        physx::PxInsertionCallback* self_ = reinterpret_cast<physx::PxInsertionCallback*>(self__pod);
        auto type = static_cast<physx::PxConcreteType::Enum>(type_pod);
        physx::PxBase* return_val = self_->buildObjectFromData(type, data);
        auto return_val_pod = reinterpret_cast<physx_PxBase*>(return_val);
        return return_val_pod;
    }

    void PxTaskManager_setCpuDispatcher(physx_PxTaskManager* self__pod, physx_PxCpuDispatcher* ref_pod) {
        physx::PxTaskManager* self_ = reinterpret_cast<physx::PxTaskManager*>(self__pod);
        physx::PxCpuDispatcher& ref = reinterpret_cast<physx::PxCpuDispatcher&>(*ref_pod);
        self_->setCpuDispatcher(ref);
    }

    physx_PxCpuDispatcher* PxTaskManager_getCpuDispatcher(physx_PxTaskManager const* self__pod) {
        physx::PxTaskManager const* self_ = reinterpret_cast<physx::PxTaskManager const*>(self__pod);
        physx::PxCpuDispatcher* return_val = self_->getCpuDispatcher();
        auto return_val_pod = reinterpret_cast<physx_PxCpuDispatcher*>(return_val);
        return return_val_pod;
    }

    void PxTaskManager_resetDependencies(physx_PxTaskManager* self__pod) {
        physx::PxTaskManager* self_ = reinterpret_cast<physx::PxTaskManager*>(self__pod);
        self_->resetDependencies();
    }

    void PxTaskManager_startSimulation(physx_PxTaskManager* self__pod) {
        physx::PxTaskManager* self_ = reinterpret_cast<physx::PxTaskManager*>(self__pod);
        self_->startSimulation();
    }

    void PxTaskManager_stopSimulation(physx_PxTaskManager* self__pod) {
        physx::PxTaskManager* self_ = reinterpret_cast<physx::PxTaskManager*>(self__pod);
        self_->stopSimulation();
    }

    void PxTaskManager_taskCompleted(physx_PxTaskManager* self__pod, physx_PxTask* task_pod) {
        physx::PxTaskManager* self_ = reinterpret_cast<physx::PxTaskManager*>(self__pod);
        physx::PxTask& task = reinterpret_cast<physx::PxTask&>(*task_pod);
        self_->taskCompleted(task);
    }

    uint32_t PxTaskManager_getNamedTask(physx_PxTaskManager* self__pod, char const* name) {
        physx::PxTaskManager* self_ = reinterpret_cast<physx::PxTaskManager*>(self__pod);
        uint32_t return_val = self_->getNamedTask(name);
        return return_val;
    }

    uint32_t PxTaskManager_submitNamedTask(physx_PxTaskManager* self__pod, physx_PxTask* task_pod, char const* name, PxTaskType type_pod) {
        physx::PxTaskManager* self_ = reinterpret_cast<physx::PxTaskManager*>(self__pod);
        physx::PxTask* task = reinterpret_cast<physx::PxTask*>(task_pod);
        auto type = static_cast<physx::PxTaskType::Enum>(type_pod);
        uint32_t return_val = self_->submitNamedTask(task, name, type);
        return return_val;
    }

    uint32_t PxTaskManager_submitUnnamedTask(physx_PxTaskManager* self__pod, physx_PxTask* task_pod, PxTaskType type_pod) {
        physx::PxTaskManager* self_ = reinterpret_cast<physx::PxTaskManager*>(self__pod);
        physx::PxTask& task = reinterpret_cast<physx::PxTask&>(*task_pod);
        auto type = static_cast<physx::PxTaskType::Enum>(type_pod);
        uint32_t return_val = self_->submitUnnamedTask(task, type);
        return return_val;
    }

    physx_PxTask* PxTaskManager_getTaskFromID(physx_PxTaskManager* self__pod, uint32_t id) {
        physx::PxTaskManager* self_ = reinterpret_cast<physx::PxTaskManager*>(self__pod);
        physx::PxTask* return_val = self_->getTaskFromID(id);
        auto return_val_pod = reinterpret_cast<physx_PxTask*>(return_val);
        return return_val_pod;
    }

    void PxTaskManager_release(physx_PxTaskManager* self__pod) {
        physx::PxTaskManager* self_ = reinterpret_cast<physx::PxTaskManager*>(self__pod);
        self_->release();
    }

    physx_PxTaskManager* PxTaskManager_createTaskManager(physx_PxErrorCallback* errorCallback_pod, physx_PxCpuDispatcher* anon_param1_pod) {
        physx::PxErrorCallback& errorCallback = reinterpret_cast<physx::PxErrorCallback&>(*errorCallback_pod);
        physx::PxCpuDispatcher* anon_param1 = reinterpret_cast<physx::PxCpuDispatcher*>(anon_param1_pod);
        physx::PxTaskManager* return_val = PxTaskManager::createTaskManager(errorCallback, anon_param1);
        auto return_val_pod = reinterpret_cast<physx_PxTaskManager*>(return_val);
        return return_val_pod;
    }

    void PxCpuDispatcher_submitTask(physx_PxCpuDispatcher* self__pod, physx_PxBaseTask* task_pod) {
        physx::PxCpuDispatcher* self_ = reinterpret_cast<physx::PxCpuDispatcher*>(self__pod);
        physx::PxBaseTask& task = reinterpret_cast<physx::PxBaseTask&>(*task_pod);
        self_->submitTask(task);
    }

    uint32_t PxCpuDispatcher_getWorkerCount(physx_PxCpuDispatcher const* self__pod) {
        physx::PxCpuDispatcher const* self_ = reinterpret_cast<physx::PxCpuDispatcher const*>(self__pod);
        uint32_t return_val = self_->getWorkerCount();
        return return_val;
    }

    void PxCpuDispatcher_delete(physx_PxCpuDispatcher* self__pod) {
        physx::PxCpuDispatcher* self_ = reinterpret_cast<physx::PxCpuDispatcher*>(self__pod);
        delete self_;
    }

    void PxBaseTask_run(physx_PxBaseTask* self__pod) {
        physx::PxBaseTask* self_ = reinterpret_cast<physx::PxBaseTask*>(self__pod);
        self_->run();
    }

    char const* PxBaseTask_getName(physx_PxBaseTask const* self__pod) {
        physx::PxBaseTask const* self_ = reinterpret_cast<physx::PxBaseTask const*>(self__pod);
        char const* return_val = self_->getName();
        return return_val;
    }

    void PxBaseTask_addReference(physx_PxBaseTask* self__pod) {
        physx::PxBaseTask* self_ = reinterpret_cast<physx::PxBaseTask*>(self__pod);
        self_->addReference();
    }

    void PxBaseTask_removeReference(physx_PxBaseTask* self__pod) {
        physx::PxBaseTask* self_ = reinterpret_cast<physx::PxBaseTask*>(self__pod);
        self_->removeReference();
    }

    int32_t PxBaseTask_getReference(physx_PxBaseTask const* self__pod) {
        physx::PxBaseTask const* self_ = reinterpret_cast<physx::PxBaseTask const*>(self__pod);
        int32_t return_val = self_->getReference();
        return return_val;
    }

    void PxBaseTask_release(physx_PxBaseTask* self__pod) {
        physx::PxBaseTask* self_ = reinterpret_cast<physx::PxBaseTask*>(self__pod);
        self_->release();
    }

    physx_PxTaskManager* PxBaseTask_getTaskManager(physx_PxBaseTask const* self__pod) {
        physx::PxBaseTask const* self_ = reinterpret_cast<physx::PxBaseTask const*>(self__pod);
        physx::PxTaskManager* return_val = self_->getTaskManager();
        auto return_val_pod = reinterpret_cast<physx_PxTaskManager*>(return_val);
        return return_val_pod;
    }

    void PxBaseTask_setContextId(physx_PxBaseTask* self__pod, uint64_t id) {
        physx::PxBaseTask* self_ = reinterpret_cast<physx::PxBaseTask*>(self__pod);
        self_->setContextId(id);
    }

    uint64_t PxBaseTask_getContextId(physx_PxBaseTask const* self__pod) {
        physx::PxBaseTask const* self_ = reinterpret_cast<physx::PxBaseTask const*>(self__pod);
        uint64_t return_val = self_->getContextId();
        return return_val;
    }

    void PxTask_release(physx_PxTask* self__pod) {
        physx::PxTask* self_ = reinterpret_cast<physx::PxTask*>(self__pod);
        self_->release();
    }

    void PxTask_finishBefore(physx_PxTask* self__pod, uint32_t taskID) {
        physx::PxTask* self_ = reinterpret_cast<physx::PxTask*>(self__pod);
        self_->finishBefore(taskID);
    }

    void PxTask_startAfter(physx_PxTask* self__pod, uint32_t taskID) {
        physx::PxTask* self_ = reinterpret_cast<physx::PxTask*>(self__pod);
        self_->startAfter(taskID);
    }

    void PxTask_addReference(physx_PxTask* self__pod) {
        physx::PxTask* self_ = reinterpret_cast<physx::PxTask*>(self__pod);
        self_->addReference();
    }

    void PxTask_removeReference(physx_PxTask* self__pod) {
        physx::PxTask* self_ = reinterpret_cast<physx::PxTask*>(self__pod);
        self_->removeReference();
    }

    int32_t PxTask_getReference(physx_PxTask const* self__pod) {
        physx::PxTask const* self_ = reinterpret_cast<physx::PxTask const*>(self__pod);
        int32_t return_val = self_->getReference();
        return return_val;
    }

    uint32_t PxTask_getTaskID(physx_PxTask const* self__pod) {
        physx::PxTask const* self_ = reinterpret_cast<physx::PxTask const*>(self__pod);
        uint32_t return_val = self_->getTaskID();
        return return_val;
    }

    void PxTask_submitted(physx_PxTask* self__pod) {
        physx::PxTask* self_ = reinterpret_cast<physx::PxTask*>(self__pod);
        self_->submitted();
    }

    void PxLightCpuTask_setContinuation(physx_PxLightCpuTask* self__pod, physx_PxTaskManager* tm_pod, physx_PxBaseTask* c_pod) {
        physx::PxLightCpuTask* self_ = reinterpret_cast<physx::PxLightCpuTask*>(self__pod);
        physx::PxTaskManager& tm = reinterpret_cast<physx::PxTaskManager&>(*tm_pod);
        physx::PxBaseTask* c = reinterpret_cast<physx::PxBaseTask*>(c_pod);
        self_->setContinuation(tm, c);
    }

    void PxLightCpuTask_setContinuation_1(physx_PxLightCpuTask* self__pod, physx_PxBaseTask* c_pod) {
        physx::PxLightCpuTask* self_ = reinterpret_cast<physx::PxLightCpuTask*>(self__pod);
        physx::PxBaseTask* c = reinterpret_cast<physx::PxBaseTask*>(c_pod);
        self_->setContinuation(c);
    }

    physx_PxBaseTask* PxLightCpuTask_getContinuation(physx_PxLightCpuTask const* self__pod) {
        physx::PxLightCpuTask const* self_ = reinterpret_cast<physx::PxLightCpuTask const*>(self__pod);
        physx::PxBaseTask* return_val = self_->getContinuation();
        auto return_val_pod = reinterpret_cast<physx_PxBaseTask*>(return_val);
        return return_val_pod;
    }

    void PxLightCpuTask_removeReference(physx_PxLightCpuTask* self__pod) {
        physx::PxLightCpuTask* self_ = reinterpret_cast<physx::PxLightCpuTask*>(self__pod);
        self_->removeReference();
    }

    int32_t PxLightCpuTask_getReference(physx_PxLightCpuTask const* self__pod) {
        physx::PxLightCpuTask const* self_ = reinterpret_cast<physx::PxLightCpuTask const*>(self__pod);
        int32_t return_val = self_->getReference();
        return return_val;
    }

    void PxLightCpuTask_addReference(physx_PxLightCpuTask* self__pod) {
        physx::PxLightCpuTask* self_ = reinterpret_cast<physx::PxLightCpuTask*>(self__pod);
        self_->addReference();
    }

    void PxLightCpuTask_release(physx_PxLightCpuTask* self__pod) {
        physx::PxLightCpuTask* self_ = reinterpret_cast<physx::PxLightCpuTask*>(self__pod);
        self_->release();
    }

    PxGeometryType PxGeometry_getType(physx_PxGeometry const* self__pod) {
        physx::PxGeometry const* self_ = reinterpret_cast<physx::PxGeometry const*>(self__pod);
        physx::PxGeometryType::Enum return_val = self_->getType();
        PxGeometryType return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxBoxGeometry PxBoxGeometry_new(float hx, float hy, float hz) {
        PxBoxGeometry return_val(hx, hy, hz);
        physx_PxBoxGeometry return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxBoxGeometry PxBoxGeometry_new_1(physx_PxVec3 halfExtents__pod) {
        physx::PxVec3 halfExtents_;
        memcpy(&halfExtents_, &halfExtents__pod, sizeof(halfExtents_));
        PxBoxGeometry return_val(halfExtents_);
        physx_PxBoxGeometry return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxBoxGeometry_isValid(physx_PxBoxGeometry const* self__pod) {
        physx::PxBoxGeometry const* self_ = reinterpret_cast<physx::PxBoxGeometry const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    void PxBVHRaycastCallback_delete(physx_PxBVHRaycastCallback* self__pod) {
        physx::PxBVHRaycastCallback* self_ = reinterpret_cast<physx::PxBVHRaycastCallback*>(self__pod);
        delete self_;
    }

    bool PxBVHRaycastCallback_reportHit(physx_PxBVHRaycastCallback* self__pod, uint32_t boundsIndex, float* distance_pod) {
        physx::PxBVHRaycastCallback* self_ = reinterpret_cast<physx::PxBVHRaycastCallback*>(self__pod);
        float& distance = *distance_pod;
        bool return_val = self_->reportHit(boundsIndex, distance);
        return return_val;
    }

    void PxBVHOverlapCallback_delete(physx_PxBVHOverlapCallback* self__pod) {
        physx::PxBVHOverlapCallback* self_ = reinterpret_cast<physx::PxBVHOverlapCallback*>(self__pod);
        delete self_;
    }

    bool PxBVHOverlapCallback_reportHit(physx_PxBVHOverlapCallback* self__pod, uint32_t boundsIndex) {
        physx::PxBVHOverlapCallback* self_ = reinterpret_cast<physx::PxBVHOverlapCallback*>(self__pod);
        bool return_val = self_->reportHit(boundsIndex);
        return return_val;
    }

    void PxBVHTraversalCallback_delete(physx_PxBVHTraversalCallback* self__pod) {
        physx::PxBVHTraversalCallback* self_ = reinterpret_cast<physx::PxBVHTraversalCallback*>(self__pod);
        delete self_;
    }

    bool PxBVHTraversalCallback_visitNode(physx_PxBVHTraversalCallback* self__pod, physx_PxBounds3 const* bounds_pod) {
        physx::PxBVHTraversalCallback* self_ = reinterpret_cast<physx::PxBVHTraversalCallback*>(self__pod);
        physx::PxBounds3 const& bounds = reinterpret_cast<physx::PxBounds3 const&>(*bounds_pod);
        bool return_val = self_->visitNode(bounds);
        return return_val;
    }

    bool PxBVHTraversalCallback_reportLeaf(physx_PxBVHTraversalCallback* self__pod, uint32_t nbPrims, uint32_t const* prims) {
        physx::PxBVHTraversalCallback* self_ = reinterpret_cast<physx::PxBVHTraversalCallback*>(self__pod);
        bool return_val = self_->reportLeaf(nbPrims, prims);
        return return_val;
    }

    bool PxBVH_raycast(physx_PxBVH const* self__pod, physx_PxVec3 const* origin_pod, physx_PxVec3 const* unitDir_pod, float maxDist, physx_PxBVHRaycastCallback* cb_pod, PxGeometryQueryFlags queryFlags_pod) {
        physx::PxBVH const* self_ = reinterpret_cast<physx::PxBVH const*>(self__pod);
        physx::PxVec3 const& origin = reinterpret_cast<physx::PxVec3 const&>(*origin_pod);
        physx::PxVec3 const& unitDir = reinterpret_cast<physx::PxVec3 const&>(*unitDir_pod);
        physx::PxBVHRaycastCallback& cb = reinterpret_cast<physx::PxBVHRaycastCallback&>(*cb_pod);
        auto queryFlags = physx::PxGeometryQueryFlags(queryFlags_pod);
        bool return_val = self_->raycast(origin, unitDir, maxDist, cb, queryFlags);
        return return_val;
    }

    bool PxBVH_sweep(physx_PxBVH const* self__pod, physx_PxGeometry const* geom_pod, physx_PxTransform const* pose_pod, physx_PxVec3 const* unitDir_pod, float maxDist, physx_PxBVHRaycastCallback* cb_pod, PxGeometryQueryFlags queryFlags_pod) {
        physx::PxBVH const* self_ = reinterpret_cast<physx::PxBVH const*>(self__pod);
        physx::PxGeometry const& geom = reinterpret_cast<physx::PxGeometry const&>(*geom_pod);
        physx::PxTransform const& pose = reinterpret_cast<physx::PxTransform const&>(*pose_pod);
        physx::PxVec3 const& unitDir = reinterpret_cast<physx::PxVec3 const&>(*unitDir_pod);
        physx::PxBVHRaycastCallback& cb = reinterpret_cast<physx::PxBVHRaycastCallback&>(*cb_pod);
        auto queryFlags = physx::PxGeometryQueryFlags(queryFlags_pod);
        bool return_val = self_->sweep(geom, pose, unitDir, maxDist, cb, queryFlags);
        return return_val;
    }

    bool PxBVH_overlap(physx_PxBVH const* self__pod, physx_PxGeometry const* geom_pod, physx_PxTransform const* pose_pod, physx_PxBVHOverlapCallback* cb_pod, PxGeometryQueryFlags queryFlags_pod) {
        physx::PxBVH const* self_ = reinterpret_cast<physx::PxBVH const*>(self__pod);
        physx::PxGeometry const& geom = reinterpret_cast<physx::PxGeometry const&>(*geom_pod);
        physx::PxTransform const& pose = reinterpret_cast<physx::PxTransform const&>(*pose_pod);
        physx::PxBVHOverlapCallback& cb = reinterpret_cast<physx::PxBVHOverlapCallback&>(*cb_pod);
        auto queryFlags = physx::PxGeometryQueryFlags(queryFlags_pod);
        bool return_val = self_->overlap(geom, pose, cb, queryFlags);
        return return_val;
    }

    bool PxBVH_cull(physx_PxBVH const* self__pod, uint32_t nbPlanes, physx_PxPlane const* planes_pod, physx_PxBVHOverlapCallback* cb_pod, PxGeometryQueryFlags queryFlags_pod) {
        physx::PxBVH const* self_ = reinterpret_cast<physx::PxBVH const*>(self__pod);
        physx::PxPlane const* planes = reinterpret_cast<physx::PxPlane const*>(planes_pod);
        physx::PxBVHOverlapCallback& cb = reinterpret_cast<physx::PxBVHOverlapCallback&>(*cb_pod);
        auto queryFlags = physx::PxGeometryQueryFlags(queryFlags_pod);
        bool return_val = self_->cull(nbPlanes, planes, cb, queryFlags);
        return return_val;
    }

    uint32_t PxBVH_getNbBounds(physx_PxBVH const* self__pod) {
        physx::PxBVH const* self_ = reinterpret_cast<physx::PxBVH const*>(self__pod);
        uint32_t return_val = self_->getNbBounds();
        return return_val;
    }

    physx_PxBounds3 const* PxBVH_getBounds(physx_PxBVH const* self__pod) {
        physx::PxBVH const* self_ = reinterpret_cast<physx::PxBVH const*>(self__pod);
        physx::PxBounds3 const* return_val = self_->getBounds();
        auto return_val_pod = reinterpret_cast<physx_PxBounds3 const*>(return_val);
        return return_val_pod;
    }

    physx_PxBounds3* PxBVH_getBoundsForModification(physx_PxBVH* self__pod) {
        physx::PxBVH* self_ = reinterpret_cast<physx::PxBVH*>(self__pod);
        physx::PxBounds3* return_val = self_->getBoundsForModification();
        auto return_val_pod = reinterpret_cast<physx_PxBounds3*>(return_val);
        return return_val_pod;
    }

    void PxBVH_refit(physx_PxBVH* self__pod) {
        physx::PxBVH* self_ = reinterpret_cast<physx::PxBVH*>(self__pod);
        self_->refit();
    }

    bool PxBVH_updateBounds(physx_PxBVH* self__pod, uint32_t boundsIndex, physx_PxBounds3 const* newBounds_pod) {
        physx::PxBVH* self_ = reinterpret_cast<physx::PxBVH*>(self__pod);
        physx::PxBounds3 const& newBounds = reinterpret_cast<physx::PxBounds3 const&>(*newBounds_pod);
        bool return_val = self_->updateBounds(boundsIndex, newBounds);
        return return_val;
    }

    void PxBVH_partialRefit(physx_PxBVH* self__pod) {
        physx::PxBVH* self_ = reinterpret_cast<physx::PxBVH*>(self__pod);
        self_->partialRefit();
    }

    bool PxBVH_traverse(physx_PxBVH const* self__pod, physx_PxBVHTraversalCallback* cb_pod) {
        physx::PxBVH const* self_ = reinterpret_cast<physx::PxBVH const*>(self__pod);
        physx::PxBVHTraversalCallback& cb = reinterpret_cast<physx::PxBVHTraversalCallback&>(*cb_pod);
        bool return_val = self_->traverse(cb);
        return return_val;
    }

    char const* PxBVH_getConcreteTypeName(physx_PxBVH const* self__pod) {
        physx::PxBVH const* self_ = reinterpret_cast<physx::PxBVH const*>(self__pod);
        char const* return_val = self_->getConcreteTypeName();
        return return_val;
    }

    physx_PxCapsuleGeometry PxCapsuleGeometry_new(float radius_, float halfHeight_) {
        PxCapsuleGeometry return_val(radius_, halfHeight_);
        physx_PxCapsuleGeometry return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxCapsuleGeometry_isValid(physx_PxCapsuleGeometry const* self__pod) {
        physx::PxCapsuleGeometry const* self_ = reinterpret_cast<physx::PxCapsuleGeometry const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    uint32_t PxConvexMesh_getNbVertices(physx_PxConvexMesh const* self__pod) {
        physx::PxConvexMesh const* self_ = reinterpret_cast<physx::PxConvexMesh const*>(self__pod);
        uint32_t return_val = self_->getNbVertices();
        return return_val;
    }

    physx_PxVec3 const* PxConvexMesh_getVertices(physx_PxConvexMesh const* self__pod) {
        physx::PxConvexMesh const* self_ = reinterpret_cast<physx::PxConvexMesh const*>(self__pod);
        physx::PxVec3 const* return_val = self_->getVertices();
        auto return_val_pod = reinterpret_cast<physx_PxVec3 const*>(return_val);
        return return_val_pod;
    }

    uint8_t const* PxConvexMesh_getIndexBuffer(physx_PxConvexMesh const* self__pod) {
        physx::PxConvexMesh const* self_ = reinterpret_cast<physx::PxConvexMesh const*>(self__pod);
        uint8_t const* return_val = self_->getIndexBuffer();
        return return_val;
    }

    uint32_t PxConvexMesh_getNbPolygons(physx_PxConvexMesh const* self__pod) {
        physx::PxConvexMesh const* self_ = reinterpret_cast<physx::PxConvexMesh const*>(self__pod);
        uint32_t return_val = self_->getNbPolygons();
        return return_val;
    }

    bool PxConvexMesh_getPolygonData(physx_PxConvexMesh const* self__pod, uint32_t index, physx_PxHullPolygon* data_pod) {
        physx::PxConvexMesh const* self_ = reinterpret_cast<physx::PxConvexMesh const*>(self__pod);
        physx::PxHullPolygon& data = reinterpret_cast<physx::PxHullPolygon&>(*data_pod);
        bool return_val = self_->getPolygonData(index, data);
        return return_val;
    }

    void PxConvexMesh_release(physx_PxConvexMesh* self__pod) {
        physx::PxConvexMesh* self_ = reinterpret_cast<physx::PxConvexMesh*>(self__pod);
        self_->release();
    }

    void PxConvexMesh_getMassInformation(physx_PxConvexMesh const* self__pod, float* mass_pod, physx_PxMat33* localInertia_pod, physx_PxVec3* localCenterOfMass_pod) {
        physx::PxConvexMesh const* self_ = reinterpret_cast<physx::PxConvexMesh const*>(self__pod);
        float& mass = *mass_pod;
        physx::PxMat33& localInertia = reinterpret_cast<physx::PxMat33&>(*localInertia_pod);
        physx::PxVec3& localCenterOfMass = reinterpret_cast<physx::PxVec3&>(*localCenterOfMass_pod);
        self_->getMassInformation(mass, localInertia, localCenterOfMass);
    }

    physx_PxBounds3 PxConvexMesh_getLocalBounds(physx_PxConvexMesh const* self__pod) {
        physx::PxConvexMesh const* self_ = reinterpret_cast<physx::PxConvexMesh const*>(self__pod);
        physx::PxBounds3 return_val = self_->getLocalBounds();
        physx_PxBounds3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    float const* PxConvexMesh_getSDF(physx_PxConvexMesh const* self__pod) {
        physx::PxConvexMesh const* self_ = reinterpret_cast<physx::PxConvexMesh const*>(self__pod);
        float const* return_val = self_->getSDF();
        return return_val;
    }

    char const* PxConvexMesh_getConcreteTypeName(physx_PxConvexMesh const* self__pod) {
        physx::PxConvexMesh const* self_ = reinterpret_cast<physx::PxConvexMesh const*>(self__pod);
        char const* return_val = self_->getConcreteTypeName();
        return return_val;
    }

    bool PxConvexMesh_isGpuCompatible(physx_PxConvexMesh const* self__pod) {
        physx::PxConvexMesh const* self_ = reinterpret_cast<physx::PxConvexMesh const*>(self__pod);
        bool return_val = self_->isGpuCompatible();
        return return_val;
    }

    physx_PxMeshScale PxMeshScale_new() {
        PxMeshScale return_val;
        physx_PxMeshScale return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxMeshScale PxMeshScale_new_1(float r) {
        PxMeshScale return_val(r);
        physx_PxMeshScale return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxMeshScale PxMeshScale_new_2(physx_PxVec3 const* s_pod) {
        physx::PxVec3 const& s = reinterpret_cast<physx::PxVec3 const&>(*s_pod);
        PxMeshScale return_val(s);
        physx_PxMeshScale return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxMeshScale PxMeshScale_new_3(physx_PxVec3 const* s_pod, physx_PxQuat const* r_pod) {
        physx::PxVec3 const& s = reinterpret_cast<physx::PxVec3 const&>(*s_pod);
        physx::PxQuat const& r = reinterpret_cast<physx::PxQuat const&>(*r_pod);
        PxMeshScale return_val(s, r);
        physx_PxMeshScale return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxMeshScale_isIdentity(physx_PxMeshScale const* self__pod) {
        physx::PxMeshScale const* self_ = reinterpret_cast<physx::PxMeshScale const*>(self__pod);
        bool return_val = self_->isIdentity();
        return return_val;
    }

    physx_PxMeshScale PxMeshScale_getInverse(physx_PxMeshScale const* self__pod) {
        physx::PxMeshScale const* self_ = reinterpret_cast<physx::PxMeshScale const*>(self__pod);
        physx::PxMeshScale return_val = self_->getInverse();
        physx_PxMeshScale return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxMat33 PxMeshScale_toMat33(physx_PxMeshScale const* self__pod) {
        physx::PxMeshScale const* self_ = reinterpret_cast<physx::PxMeshScale const*>(self__pod);
        physx::PxMat33 return_val = self_->toMat33();
        physx_PxMat33 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxMeshScale_hasNegativeDeterminant(physx_PxMeshScale const* self__pod) {
        physx::PxMeshScale const* self_ = reinterpret_cast<physx::PxMeshScale const*>(self__pod);
        bool return_val = self_->hasNegativeDeterminant();
        return return_val;
    }

    physx_PxVec3 PxMeshScale_transform(physx_PxMeshScale const* self__pod, physx_PxVec3 const* v_pod) {
        physx::PxMeshScale const* self_ = reinterpret_cast<physx::PxMeshScale const*>(self__pod);
        physx::PxVec3 const& v = reinterpret_cast<physx::PxVec3 const&>(*v_pod);
        physx::PxVec3 return_val = self_->transform(v);
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxMeshScale_isValidForTriangleMesh(physx_PxMeshScale const* self__pod) {
        physx::PxMeshScale const* self_ = reinterpret_cast<physx::PxMeshScale const*>(self__pod);
        bool return_val = self_->isValidForTriangleMesh();
        return return_val;
    }

    bool PxMeshScale_isValidForConvexMesh(physx_PxMeshScale const* self__pod) {
        physx::PxMeshScale const* self_ = reinterpret_cast<physx::PxMeshScale const*>(self__pod);
        bool return_val = self_->isValidForConvexMesh();
        return return_val;
    }

    physx_PxConvexMeshGeometry PxConvexMeshGeometry_new(physx_PxConvexMesh* mesh_pod, physx_PxMeshScale const* scaling_pod, PxConvexMeshGeometryFlags flags_pod) {
        physx::PxConvexMesh* mesh = reinterpret_cast<physx::PxConvexMesh*>(mesh_pod);
        physx::PxMeshScale const& scaling = reinterpret_cast<physx::PxMeshScale const&>(*scaling_pod);
        auto flags = physx::PxConvexMeshGeometryFlags(flags_pod);
        PxConvexMeshGeometry return_val(mesh, scaling, flags);
        physx_PxConvexMeshGeometry return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxConvexMeshGeometry_isValid(physx_PxConvexMeshGeometry const* self__pod) {
        physx::PxConvexMeshGeometry const* self_ = reinterpret_cast<physx::PxConvexMeshGeometry const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    physx_PxSphereGeometry PxSphereGeometry_new(float ir) {
        PxSphereGeometry return_val(ir);
        physx_PxSphereGeometry return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxSphereGeometry_isValid(physx_PxSphereGeometry const* self__pod) {
        physx::PxSphereGeometry const* self_ = reinterpret_cast<physx::PxSphereGeometry const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    physx_PxPlaneGeometry PxPlaneGeometry_new() {
        PxPlaneGeometry return_val;
        physx_PxPlaneGeometry return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxPlaneGeometry_isValid(physx_PxPlaneGeometry const* self__pod) {
        physx::PxPlaneGeometry const* self_ = reinterpret_cast<physx::PxPlaneGeometry const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    physx_PxTriangleMeshGeometry PxTriangleMeshGeometry_new(physx_PxTriangleMesh* mesh_pod, physx_PxMeshScale const* scaling_pod, PxMeshGeometryFlags flags_pod) {
        physx::PxTriangleMesh* mesh = reinterpret_cast<physx::PxTriangleMesh*>(mesh_pod);
        physx::PxMeshScale const& scaling = reinterpret_cast<physx::PxMeshScale const&>(*scaling_pod);
        auto flags = physx::PxMeshGeometryFlags(flags_pod);
        PxTriangleMeshGeometry return_val(mesh, scaling, flags);
        physx_PxTriangleMeshGeometry return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxTriangleMeshGeometry_isValid(physx_PxTriangleMeshGeometry const* self__pod) {
        physx::PxTriangleMeshGeometry const* self_ = reinterpret_cast<physx::PxTriangleMeshGeometry const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    physx_PxHeightFieldGeometry PxHeightFieldGeometry_new(physx_PxHeightField* hf_pod, PxMeshGeometryFlags flags_pod, float heightScale_, float rowScale_, float columnScale_) {
        physx::PxHeightField* hf = reinterpret_cast<physx::PxHeightField*>(hf_pod);
        auto flags = physx::PxMeshGeometryFlags(flags_pod);
        PxHeightFieldGeometry return_val(hf, flags, heightScale_, rowScale_, columnScale_);
        physx_PxHeightFieldGeometry return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxHeightFieldGeometry_isValid(physx_PxHeightFieldGeometry const* self__pod) {
        physx::PxHeightFieldGeometry const* self_ = reinterpret_cast<physx::PxHeightFieldGeometry const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    physx_PxParticleSystemGeometry PxParticleSystemGeometry_new() {
        PxParticleSystemGeometry return_val;
        physx_PxParticleSystemGeometry return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxParticleSystemGeometry_isValid(physx_PxParticleSystemGeometry const* self__pod) {
        physx::PxParticleSystemGeometry const* self_ = reinterpret_cast<physx::PxParticleSystemGeometry const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    physx_PxHairSystemGeometry PxHairSystemGeometry_new() {
        PxHairSystemGeometry return_val;
        physx_PxHairSystemGeometry return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxHairSystemGeometry_isValid(physx_PxHairSystemGeometry const* self__pod) {
        physx::PxHairSystemGeometry const* self_ = reinterpret_cast<physx::PxHairSystemGeometry const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    physx_PxTetrahedronMeshGeometry PxTetrahedronMeshGeometry_new(physx_PxTetrahedronMesh* mesh_pod) {
        physx::PxTetrahedronMesh* mesh = reinterpret_cast<physx::PxTetrahedronMesh*>(mesh_pod);
        PxTetrahedronMeshGeometry return_val(mesh);
        physx_PxTetrahedronMeshGeometry return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxTetrahedronMeshGeometry_isValid(physx_PxTetrahedronMeshGeometry const* self__pod) {
        physx::PxTetrahedronMeshGeometry const* self_ = reinterpret_cast<physx::PxTetrahedronMeshGeometry const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    physx_PxQueryHit PxQueryHit_new() {
        PxQueryHit return_val;
        physx_PxQueryHit return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxLocationHit PxLocationHit_new() {
        PxLocationHit return_val;
        physx_PxLocationHit return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxLocationHit_hadInitialOverlap(physx_PxLocationHit const* self__pod) {
        physx::PxLocationHit const* self_ = reinterpret_cast<physx::PxLocationHit const*>(self__pod);
        bool return_val = self_->hadInitialOverlap();
        return return_val;
    }

    physx_PxGeomRaycastHit PxGeomRaycastHit_new() {
        PxGeomRaycastHit return_val;
        physx_PxGeomRaycastHit return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxGeomOverlapHit PxGeomOverlapHit_new() {
        PxGeomOverlapHit return_val;
        physx_PxGeomOverlapHit return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxGeomSweepHit PxGeomSweepHit_new() {
        PxGeomSweepHit return_val;
        physx_PxGeomSweepHit return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxGeomIndexPair PxGeomIndexPair_new() {
        PxGeomIndexPair return_val;
        physx_PxGeomIndexPair return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxGeomIndexPair PxGeomIndexPair_new_1(uint32_t _id0, uint32_t _id1) {
        PxGeomIndexPair return_val(_id0, _id1);
        physx_PxGeomIndexPair return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    uint32_t phys_PxCustomGeometry_getUniqueID() {
        uint32_t return_val = PxCustomGeometry_getUniqueID();
        return return_val;
    }

    physx_PxCustomGeometryType PxCustomGeometryType_new() {
        PxCustomGeometryType return_val;
        physx_PxCustomGeometryType return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxCustomGeometryType PxCustomGeometryType_INVALID() {
        physx::PxCustomGeometryType return_val = PxCustomGeometryType::INVALID();
        physx_PxCustomGeometryType return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxCustomGeometryType PxCustomGeometryCallbacks_getCustomType(physx_PxCustomGeometryCallbacks const* self__pod) {
        physx::PxCustomGeometryCallbacks const* self_ = reinterpret_cast<physx::PxCustomGeometryCallbacks const*>(self__pod);
        physx::PxCustomGeometryType return_val = self_->getCustomType();
        physx_PxCustomGeometryType return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxBounds3 PxCustomGeometryCallbacks_getLocalBounds(physx_PxCustomGeometryCallbacks const* self__pod, physx_PxGeometry const* geometry_pod) {
        physx::PxCustomGeometryCallbacks const* self_ = reinterpret_cast<physx::PxCustomGeometryCallbacks const*>(self__pod);
        physx::PxGeometry const& geometry = reinterpret_cast<physx::PxGeometry const&>(*geometry_pod);
        physx::PxBounds3 return_val = self_->getLocalBounds(geometry);
        physx_PxBounds3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    uint32_t PxCustomGeometryCallbacks_raycast(physx_PxCustomGeometryCallbacks const* self__pod, physx_PxVec3 const* origin_pod, physx_PxVec3 const* unitDir_pod, physx_PxGeometry const* geom_pod, physx_PxTransform const* pose_pod, float maxDist, PxHitFlags hitFlags_pod, uint32_t maxHits, physx_PxGeomRaycastHit* rayHits_pod, uint32_t stride, physx_PxQueryThreadContext* threadContext_pod) {
        physx::PxCustomGeometryCallbacks const* self_ = reinterpret_cast<physx::PxCustomGeometryCallbacks const*>(self__pod);
        physx::PxVec3 const& origin = reinterpret_cast<physx::PxVec3 const&>(*origin_pod);
        physx::PxVec3 const& unitDir = reinterpret_cast<physx::PxVec3 const&>(*unitDir_pod);
        physx::PxGeometry const& geom = reinterpret_cast<physx::PxGeometry const&>(*geom_pod);
        physx::PxTransform const& pose = reinterpret_cast<physx::PxTransform const&>(*pose_pod);
        auto hitFlags = physx::PxHitFlags(hitFlags_pod);
        physx::PxGeomRaycastHit* rayHits = reinterpret_cast<physx::PxGeomRaycastHit*>(rayHits_pod);
        physx::PxQueryThreadContext* threadContext = reinterpret_cast<physx::PxQueryThreadContext*>(threadContext_pod);
        uint32_t return_val = self_->raycast(origin, unitDir, geom, pose, maxDist, hitFlags, maxHits, rayHits, stride, threadContext);
        return return_val;
    }

    bool PxCustomGeometryCallbacks_overlap(physx_PxCustomGeometryCallbacks const* self__pod, physx_PxGeometry const* geom0_pod, physx_PxTransform const* pose0_pod, physx_PxGeometry const* geom1_pod, physx_PxTransform const* pose1_pod, physx_PxQueryThreadContext* threadContext_pod) {
        physx::PxCustomGeometryCallbacks const* self_ = reinterpret_cast<physx::PxCustomGeometryCallbacks const*>(self__pod);
        physx::PxGeometry const& geom0 = reinterpret_cast<physx::PxGeometry const&>(*geom0_pod);
        physx::PxTransform const& pose0 = reinterpret_cast<physx::PxTransform const&>(*pose0_pod);
        physx::PxGeometry const& geom1 = reinterpret_cast<physx::PxGeometry const&>(*geom1_pod);
        physx::PxTransform const& pose1 = reinterpret_cast<physx::PxTransform const&>(*pose1_pod);
        physx::PxQueryThreadContext* threadContext = reinterpret_cast<physx::PxQueryThreadContext*>(threadContext_pod);
        bool return_val = self_->overlap(geom0, pose0, geom1, pose1, threadContext);
        return return_val;
    }

    bool PxCustomGeometryCallbacks_sweep(physx_PxCustomGeometryCallbacks const* self__pod, physx_PxVec3 const* unitDir_pod, float maxDist, physx_PxGeometry const* geom0_pod, physx_PxTransform const* pose0_pod, physx_PxGeometry const* geom1_pod, physx_PxTransform const* pose1_pod, physx_PxGeomSweepHit* sweepHit_pod, PxHitFlags hitFlags_pod, float inflation, physx_PxQueryThreadContext* threadContext_pod) {
        physx::PxCustomGeometryCallbacks const* self_ = reinterpret_cast<physx::PxCustomGeometryCallbacks const*>(self__pod);
        physx::PxVec3 const& unitDir = reinterpret_cast<physx::PxVec3 const&>(*unitDir_pod);
        physx::PxGeometry const& geom0 = reinterpret_cast<physx::PxGeometry const&>(*geom0_pod);
        physx::PxTransform const& pose0 = reinterpret_cast<physx::PxTransform const&>(*pose0_pod);
        physx::PxGeometry const& geom1 = reinterpret_cast<physx::PxGeometry const&>(*geom1_pod);
        physx::PxTransform const& pose1 = reinterpret_cast<physx::PxTransform const&>(*pose1_pod);
        physx::PxGeomSweepHit& sweepHit = reinterpret_cast<physx::PxGeomSweepHit&>(*sweepHit_pod);
        auto hitFlags = physx::PxHitFlags(hitFlags_pod);
        physx::PxQueryThreadContext* threadContext = reinterpret_cast<physx::PxQueryThreadContext*>(threadContext_pod);
        bool return_val = self_->sweep(unitDir, maxDist, geom0, pose0, geom1, pose1, sweepHit, hitFlags, inflation, threadContext);
        return return_val;
    }

    void PxCustomGeometryCallbacks_computeMassProperties(physx_PxCustomGeometryCallbacks const* self__pod, physx_PxGeometry const* geometry_pod, physx_PxMassProperties* massProperties_pod) {
        physx::PxCustomGeometryCallbacks const* self_ = reinterpret_cast<physx::PxCustomGeometryCallbacks const*>(self__pod);
        physx::PxGeometry const& geometry = reinterpret_cast<physx::PxGeometry const&>(*geometry_pod);
        physx::PxMassProperties& massProperties = reinterpret_cast<physx::PxMassProperties&>(*massProperties_pod);
        self_->computeMassProperties(geometry, massProperties);
    }

    bool PxCustomGeometryCallbacks_usePersistentContactManifold(physx_PxCustomGeometryCallbacks const* self__pod, physx_PxGeometry const* geometry_pod, float* breakingThreshold_pod) {
        physx::PxCustomGeometryCallbacks const* self_ = reinterpret_cast<physx::PxCustomGeometryCallbacks const*>(self__pod);
        physx::PxGeometry const& geometry = reinterpret_cast<physx::PxGeometry const&>(*geometry_pod);
        float& breakingThreshold = *breakingThreshold_pod;
        bool return_val = self_->usePersistentContactManifold(geometry, breakingThreshold);
        return return_val;
    }

    void PxCustomGeometryCallbacks_delete(physx_PxCustomGeometryCallbacks* self__pod) {
        physx::PxCustomGeometryCallbacks* self_ = reinterpret_cast<physx::PxCustomGeometryCallbacks*>(self__pod);
        delete self_;
    }

    physx_PxCustomGeometry PxCustomGeometry_new() {
        PxCustomGeometry return_val;
        physx_PxCustomGeometry return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxCustomGeometry PxCustomGeometry_new_1(physx_PxCustomGeometryCallbacks* _callbacks_pod) {
        physx::PxCustomGeometryCallbacks& _callbacks = reinterpret_cast<physx::PxCustomGeometryCallbacks&>(*_callbacks_pod);
        PxCustomGeometry return_val(_callbacks);
        physx_PxCustomGeometry return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxCustomGeometry_isValid(physx_PxCustomGeometry const* self__pod) {
        physx::PxCustomGeometry const* self_ = reinterpret_cast<physx::PxCustomGeometry const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    physx_PxCustomGeometryType PxCustomGeometry_getCustomType(physx_PxCustomGeometry const* self__pod) {
        physx::PxCustomGeometry const* self_ = reinterpret_cast<physx::PxCustomGeometry const*>(self__pod);
        physx::PxCustomGeometryType return_val = self_->getCustomType();
        physx_PxCustomGeometryType return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    PxGeometryType PxGeometryHolder_getType(physx_PxGeometryHolder const* self__pod) {
        physx::PxGeometryHolder const* self_ = reinterpret_cast<physx::PxGeometryHolder const*>(self__pod);
        physx::PxGeometryType::Enum return_val = self_->getType();
        PxGeometryType return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxGeometry* PxGeometryHolder_any(physx_PxGeometryHolder* self__pod) {
        physx::PxGeometryHolder* self_ = reinterpret_cast<physx::PxGeometryHolder*>(self__pod);
        physx::PxGeometry& return_val = self_->any();
        auto return_val_pod = reinterpret_cast<physx_PxGeometry*>(&return_val);
        return return_val_pod;
    }

    physx_PxGeometry const* PxGeometryHolder_any_1(physx_PxGeometryHolder const* self__pod) {
        physx::PxGeometryHolder const* self_ = reinterpret_cast<physx::PxGeometryHolder const*>(self__pod);
        physx::PxGeometry const& return_val = self_->any();
        auto return_val_pod = reinterpret_cast<physx_PxGeometry const*>(&return_val);
        return return_val_pod;
    }

    physx_PxSphereGeometry* PxGeometryHolder_sphere(physx_PxGeometryHolder* self__pod) {
        physx::PxGeometryHolder* self_ = reinterpret_cast<physx::PxGeometryHolder*>(self__pod);
        physx::PxSphereGeometry& return_val = self_->sphere();
        auto return_val_pod = reinterpret_cast<physx_PxSphereGeometry*>(&return_val);
        return return_val_pod;
    }

    physx_PxSphereGeometry const* PxGeometryHolder_sphere_1(physx_PxGeometryHolder const* self__pod) {
        physx::PxGeometryHolder const* self_ = reinterpret_cast<physx::PxGeometryHolder const*>(self__pod);
        physx::PxSphereGeometry const& return_val = self_->sphere();
        auto return_val_pod = reinterpret_cast<physx_PxSphereGeometry const*>(&return_val);
        return return_val_pod;
    }

    physx_PxPlaneGeometry* PxGeometryHolder_plane(physx_PxGeometryHolder* self__pod) {
        physx::PxGeometryHolder* self_ = reinterpret_cast<physx::PxGeometryHolder*>(self__pod);
        physx::PxPlaneGeometry& return_val = self_->plane();
        auto return_val_pod = reinterpret_cast<physx_PxPlaneGeometry*>(&return_val);
        return return_val_pod;
    }

    physx_PxPlaneGeometry const* PxGeometryHolder_plane_1(physx_PxGeometryHolder const* self__pod) {
        physx::PxGeometryHolder const* self_ = reinterpret_cast<physx::PxGeometryHolder const*>(self__pod);
        physx::PxPlaneGeometry const& return_val = self_->plane();
        auto return_val_pod = reinterpret_cast<physx_PxPlaneGeometry const*>(&return_val);
        return return_val_pod;
    }

    physx_PxCapsuleGeometry* PxGeometryHolder_capsule(physx_PxGeometryHolder* self__pod) {
        physx::PxGeometryHolder* self_ = reinterpret_cast<physx::PxGeometryHolder*>(self__pod);
        physx::PxCapsuleGeometry& return_val = self_->capsule();
        auto return_val_pod = reinterpret_cast<physx_PxCapsuleGeometry*>(&return_val);
        return return_val_pod;
    }

    physx_PxCapsuleGeometry const* PxGeometryHolder_capsule_1(physx_PxGeometryHolder const* self__pod) {
        physx::PxGeometryHolder const* self_ = reinterpret_cast<physx::PxGeometryHolder const*>(self__pod);
        physx::PxCapsuleGeometry const& return_val = self_->capsule();
        auto return_val_pod = reinterpret_cast<physx_PxCapsuleGeometry const*>(&return_val);
        return return_val_pod;
    }

    physx_PxBoxGeometry* PxGeometryHolder_box(physx_PxGeometryHolder* self__pod) {
        physx::PxGeometryHolder* self_ = reinterpret_cast<physx::PxGeometryHolder*>(self__pod);
        physx::PxBoxGeometry& return_val = self_->box();
        auto return_val_pod = reinterpret_cast<physx_PxBoxGeometry*>(&return_val);
        return return_val_pod;
    }

    physx_PxBoxGeometry const* PxGeometryHolder_box_1(physx_PxGeometryHolder const* self__pod) {
        physx::PxGeometryHolder const* self_ = reinterpret_cast<physx::PxGeometryHolder const*>(self__pod);
        physx::PxBoxGeometry const& return_val = self_->box();
        auto return_val_pod = reinterpret_cast<physx_PxBoxGeometry const*>(&return_val);
        return return_val_pod;
    }

    physx_PxConvexMeshGeometry* PxGeometryHolder_convexMesh(physx_PxGeometryHolder* self__pod) {
        physx::PxGeometryHolder* self_ = reinterpret_cast<physx::PxGeometryHolder*>(self__pod);
        physx::PxConvexMeshGeometry& return_val = self_->convexMesh();
        auto return_val_pod = reinterpret_cast<physx_PxConvexMeshGeometry*>(&return_val);
        return return_val_pod;
    }

    physx_PxConvexMeshGeometry const* PxGeometryHolder_convexMesh_1(physx_PxGeometryHolder const* self__pod) {
        physx::PxGeometryHolder const* self_ = reinterpret_cast<physx::PxGeometryHolder const*>(self__pod);
        physx::PxConvexMeshGeometry const& return_val = self_->convexMesh();
        auto return_val_pod = reinterpret_cast<physx_PxConvexMeshGeometry const*>(&return_val);
        return return_val_pod;
    }

    physx_PxTetrahedronMeshGeometry* PxGeometryHolder_tetMesh(physx_PxGeometryHolder* self__pod) {
        physx::PxGeometryHolder* self_ = reinterpret_cast<physx::PxGeometryHolder*>(self__pod);
        physx::PxTetrahedronMeshGeometry& return_val = self_->tetMesh();
        auto return_val_pod = reinterpret_cast<physx_PxTetrahedronMeshGeometry*>(&return_val);
        return return_val_pod;
    }

    physx_PxTetrahedronMeshGeometry const* PxGeometryHolder_tetMesh_1(physx_PxGeometryHolder const* self__pod) {
        physx::PxGeometryHolder const* self_ = reinterpret_cast<physx::PxGeometryHolder const*>(self__pod);
        physx::PxTetrahedronMeshGeometry const& return_val = self_->tetMesh();
        auto return_val_pod = reinterpret_cast<physx_PxTetrahedronMeshGeometry const*>(&return_val);
        return return_val_pod;
    }

    physx_PxTriangleMeshGeometry* PxGeometryHolder_triangleMesh(physx_PxGeometryHolder* self__pod) {
        physx::PxGeometryHolder* self_ = reinterpret_cast<physx::PxGeometryHolder*>(self__pod);
        physx::PxTriangleMeshGeometry& return_val = self_->triangleMesh();
        auto return_val_pod = reinterpret_cast<physx_PxTriangleMeshGeometry*>(&return_val);
        return return_val_pod;
    }

    physx_PxTriangleMeshGeometry const* PxGeometryHolder_triangleMesh_1(physx_PxGeometryHolder const* self__pod) {
        physx::PxGeometryHolder const* self_ = reinterpret_cast<physx::PxGeometryHolder const*>(self__pod);
        physx::PxTriangleMeshGeometry const& return_val = self_->triangleMesh();
        auto return_val_pod = reinterpret_cast<physx_PxTriangleMeshGeometry const*>(&return_val);
        return return_val_pod;
    }

    physx_PxHeightFieldGeometry* PxGeometryHolder_heightField(physx_PxGeometryHolder* self__pod) {
        physx::PxGeometryHolder* self_ = reinterpret_cast<physx::PxGeometryHolder*>(self__pod);
        physx::PxHeightFieldGeometry& return_val = self_->heightField();
        auto return_val_pod = reinterpret_cast<physx_PxHeightFieldGeometry*>(&return_val);
        return return_val_pod;
    }

    physx_PxHeightFieldGeometry const* PxGeometryHolder_heightField_1(physx_PxGeometryHolder const* self__pod) {
        physx::PxGeometryHolder const* self_ = reinterpret_cast<physx::PxGeometryHolder const*>(self__pod);
        physx::PxHeightFieldGeometry const& return_val = self_->heightField();
        auto return_val_pod = reinterpret_cast<physx_PxHeightFieldGeometry const*>(&return_val);
        return return_val_pod;
    }

    physx_PxParticleSystemGeometry* PxGeometryHolder_particleSystem(physx_PxGeometryHolder* self__pod) {
        physx::PxGeometryHolder* self_ = reinterpret_cast<physx::PxGeometryHolder*>(self__pod);
        physx::PxParticleSystemGeometry& return_val = self_->particleSystem();
        auto return_val_pod = reinterpret_cast<physx_PxParticleSystemGeometry*>(&return_val);
        return return_val_pod;
    }

    physx_PxParticleSystemGeometry const* PxGeometryHolder_particleSystem_1(physx_PxGeometryHolder const* self__pod) {
        physx::PxGeometryHolder const* self_ = reinterpret_cast<physx::PxGeometryHolder const*>(self__pod);
        physx::PxParticleSystemGeometry const& return_val = self_->particleSystem();
        auto return_val_pod = reinterpret_cast<physx_PxParticleSystemGeometry const*>(&return_val);
        return return_val_pod;
    }

    physx_PxHairSystemGeometry* PxGeometryHolder_hairSystem(physx_PxGeometryHolder* self__pod) {
        physx::PxGeometryHolder* self_ = reinterpret_cast<physx::PxGeometryHolder*>(self__pod);
        physx::PxHairSystemGeometry& return_val = self_->hairSystem();
        auto return_val_pod = reinterpret_cast<physx_PxHairSystemGeometry*>(&return_val);
        return return_val_pod;
    }

    physx_PxHairSystemGeometry const* PxGeometryHolder_hairSystem_1(physx_PxGeometryHolder const* self__pod) {
        physx::PxGeometryHolder const* self_ = reinterpret_cast<physx::PxGeometryHolder const*>(self__pod);
        physx::PxHairSystemGeometry const& return_val = self_->hairSystem();
        auto return_val_pod = reinterpret_cast<physx_PxHairSystemGeometry const*>(&return_val);
        return return_val_pod;
    }

    physx_PxCustomGeometry* PxGeometryHolder_custom(physx_PxGeometryHolder* self__pod) {
        physx::PxGeometryHolder* self_ = reinterpret_cast<physx::PxGeometryHolder*>(self__pod);
        physx::PxCustomGeometry& return_val = self_->custom();
        auto return_val_pod = reinterpret_cast<physx_PxCustomGeometry*>(&return_val);
        return return_val_pod;
    }

    physx_PxCustomGeometry const* PxGeometryHolder_custom_1(physx_PxGeometryHolder const* self__pod) {
        physx::PxGeometryHolder const* self_ = reinterpret_cast<physx::PxGeometryHolder const*>(self__pod);
        physx::PxCustomGeometry const& return_val = self_->custom();
        auto return_val_pod = reinterpret_cast<physx_PxCustomGeometry const*>(&return_val);
        return return_val_pod;
    }

    void PxGeometryHolder_storeAny(physx_PxGeometryHolder* self__pod, physx_PxGeometry const* geometry_pod) {
        physx::PxGeometryHolder* self_ = reinterpret_cast<physx::PxGeometryHolder*>(self__pod);
        physx::PxGeometry const& geometry = reinterpret_cast<physx::PxGeometry const&>(*geometry_pod);
        self_->storeAny(geometry);
    }

    physx_PxGeometryHolder PxGeometryHolder_new() {
        PxGeometryHolder return_val;
        physx_PxGeometryHolder return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxGeometryHolder PxGeometryHolder_new_1(physx_PxGeometry const* geometry_pod) {
        physx::PxGeometry const& geometry = reinterpret_cast<physx::PxGeometry const&>(*geometry_pod);
        PxGeometryHolder return_val(geometry);
        physx_PxGeometryHolder return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    uint32_t PxGeometryQuery_raycast(physx_PxVec3 const* origin_pod, physx_PxVec3 const* unitDir_pod, physx_PxGeometry const* geom_pod, physx_PxTransform const* pose_pod, float maxDist, PxHitFlags hitFlags_pod, uint32_t maxHits, physx_PxGeomRaycastHit* rayHits_pod, uint32_t stride, PxGeometryQueryFlags queryFlags_pod, physx_PxQueryThreadContext* threadContext_pod) {
        physx::PxVec3 const& origin = reinterpret_cast<physx::PxVec3 const&>(*origin_pod);
        physx::PxVec3 const& unitDir = reinterpret_cast<physx::PxVec3 const&>(*unitDir_pod);
        physx::PxGeometry const& geom = reinterpret_cast<physx::PxGeometry const&>(*geom_pod);
        physx::PxTransform const& pose = reinterpret_cast<physx::PxTransform const&>(*pose_pod);
        auto hitFlags = physx::PxHitFlags(hitFlags_pod);
        physx::PxGeomRaycastHit* rayHits = reinterpret_cast<physx::PxGeomRaycastHit*>(rayHits_pod);
        auto queryFlags = physx::PxGeometryQueryFlags(queryFlags_pod);
        physx::PxQueryThreadContext* threadContext = reinterpret_cast<physx::PxQueryThreadContext*>(threadContext_pod);
        uint32_t return_val = PxGeometryQuery::raycast(origin, unitDir, geom, pose, maxDist, hitFlags, maxHits, rayHits, stride, queryFlags, threadContext);
        return return_val;
    }

    bool PxGeometryQuery_overlap(physx_PxGeometry const* geom0_pod, physx_PxTransform const* pose0_pod, physx_PxGeometry const* geom1_pod, physx_PxTransform const* pose1_pod, PxGeometryQueryFlags queryFlags_pod, physx_PxQueryThreadContext* threadContext_pod) {
        physx::PxGeometry const& geom0 = reinterpret_cast<physx::PxGeometry const&>(*geom0_pod);
        physx::PxTransform const& pose0 = reinterpret_cast<physx::PxTransform const&>(*pose0_pod);
        physx::PxGeometry const& geom1 = reinterpret_cast<physx::PxGeometry const&>(*geom1_pod);
        physx::PxTransform const& pose1 = reinterpret_cast<physx::PxTransform const&>(*pose1_pod);
        auto queryFlags = physx::PxGeometryQueryFlags(queryFlags_pod);
        physx::PxQueryThreadContext* threadContext = reinterpret_cast<physx::PxQueryThreadContext*>(threadContext_pod);
        bool return_val = PxGeometryQuery::overlap(geom0, pose0, geom1, pose1, queryFlags, threadContext);
        return return_val;
    }

    bool PxGeometryQuery_sweep(physx_PxVec3 const* unitDir_pod, float maxDist, physx_PxGeometry const* geom0_pod, physx_PxTransform const* pose0_pod, physx_PxGeometry const* geom1_pod, physx_PxTransform const* pose1_pod, physx_PxGeomSweepHit* sweepHit_pod, PxHitFlags hitFlags_pod, float inflation, PxGeometryQueryFlags queryFlags_pod, physx_PxQueryThreadContext* threadContext_pod) {
        physx::PxVec3 const& unitDir = reinterpret_cast<physx::PxVec3 const&>(*unitDir_pod);
        physx::PxGeometry const& geom0 = reinterpret_cast<physx::PxGeometry const&>(*geom0_pod);
        physx::PxTransform const& pose0 = reinterpret_cast<physx::PxTransform const&>(*pose0_pod);
        physx::PxGeometry const& geom1 = reinterpret_cast<physx::PxGeometry const&>(*geom1_pod);
        physx::PxTransform const& pose1 = reinterpret_cast<physx::PxTransform const&>(*pose1_pod);
        physx::PxGeomSweepHit& sweepHit = reinterpret_cast<physx::PxGeomSweepHit&>(*sweepHit_pod);
        auto hitFlags = physx::PxHitFlags(hitFlags_pod);
        auto queryFlags = physx::PxGeometryQueryFlags(queryFlags_pod);
        physx::PxQueryThreadContext* threadContext = reinterpret_cast<physx::PxQueryThreadContext*>(threadContext_pod);
        bool return_val = PxGeometryQuery::sweep(unitDir, maxDist, geom0, pose0, geom1, pose1, sweepHit, hitFlags, inflation, queryFlags, threadContext);
        return return_val;
    }

    bool PxGeometryQuery_computePenetration(physx_PxVec3* direction_pod, float* depth_pod, physx_PxGeometry const* geom0_pod, physx_PxTransform const* pose0_pod, physx_PxGeometry const* geom1_pod, physx_PxTransform const* pose1_pod, PxGeometryQueryFlags queryFlags_pod) {
        physx::PxVec3& direction = reinterpret_cast<physx::PxVec3&>(*direction_pod);
        float& depth = *depth_pod;
        physx::PxGeometry const& geom0 = reinterpret_cast<physx::PxGeometry const&>(*geom0_pod);
        physx::PxTransform const& pose0 = reinterpret_cast<physx::PxTransform const&>(*pose0_pod);
        physx::PxGeometry const& geom1 = reinterpret_cast<physx::PxGeometry const&>(*geom1_pod);
        physx::PxTransform const& pose1 = reinterpret_cast<physx::PxTransform const&>(*pose1_pod);
        auto queryFlags = physx::PxGeometryQueryFlags(queryFlags_pod);
        bool return_val = PxGeometryQuery::computePenetration(direction, depth, geom0, pose0, geom1, pose1, queryFlags);
        return return_val;
    }

    float PxGeometryQuery_pointDistance(physx_PxVec3 const* point_pod, physx_PxGeometry const* geom_pod, physx_PxTransform const* pose_pod, physx_PxVec3* closestPoint_pod, uint32_t* closestIndex, PxGeometryQueryFlags queryFlags_pod) {
        physx::PxVec3 const& point = reinterpret_cast<physx::PxVec3 const&>(*point_pod);
        physx::PxGeometry const& geom = reinterpret_cast<physx::PxGeometry const&>(*geom_pod);
        physx::PxTransform const& pose = reinterpret_cast<physx::PxTransform const&>(*pose_pod);
        physx::PxVec3* closestPoint = reinterpret_cast<physx::PxVec3*>(closestPoint_pod);
        auto queryFlags = physx::PxGeometryQueryFlags(queryFlags_pod);
        float return_val = PxGeometryQuery::pointDistance(point, geom, pose, closestPoint, closestIndex, queryFlags);
        return return_val;
    }

    void PxGeometryQuery_computeGeomBounds(physx_PxBounds3* bounds_pod, physx_PxGeometry const* geom_pod, physx_PxTransform const* pose_pod, float offset, float inflation, PxGeometryQueryFlags queryFlags_pod) {
        physx::PxBounds3& bounds = reinterpret_cast<physx::PxBounds3&>(*bounds_pod);
        physx::PxGeometry const& geom = reinterpret_cast<physx::PxGeometry const&>(*geom_pod);
        physx::PxTransform const& pose = reinterpret_cast<physx::PxTransform const&>(*pose_pod);
        auto queryFlags = physx::PxGeometryQueryFlags(queryFlags_pod);
        PxGeometryQuery::computeGeomBounds(bounds, geom, pose, offset, inflation, queryFlags);
    }

    bool PxGeometryQuery_isValid(physx_PxGeometry const* geom_pod) {
        physx::PxGeometry const& geom = reinterpret_cast<physx::PxGeometry const&>(*geom_pod);
        bool return_val = PxGeometryQuery::isValid(geom);
        return return_val;
    }

    uint8_t PxHeightFieldSample_tessFlag(physx_PxHeightFieldSample const* self__pod) {
        physx::PxHeightFieldSample const* self_ = reinterpret_cast<physx::PxHeightFieldSample const*>(self__pod);
        uint8_t return_val = self_->tessFlag();
        return return_val;
    }

    void PxHeightFieldSample_setTessFlag(physx_PxHeightFieldSample* self__pod) {
        physx::PxHeightFieldSample* self_ = reinterpret_cast<physx::PxHeightFieldSample*>(self__pod);
        self_->setTessFlag();
    }

    void PxHeightFieldSample_clearTessFlag(physx_PxHeightFieldSample* self__pod) {
        physx::PxHeightFieldSample* self_ = reinterpret_cast<physx::PxHeightFieldSample*>(self__pod);
        self_->clearTessFlag();
    }

    void PxHeightField_release(physx_PxHeightField* self__pod) {
        physx::PxHeightField* self_ = reinterpret_cast<physx::PxHeightField*>(self__pod);
        self_->release();
    }

    uint32_t PxHeightField_saveCells(physx_PxHeightField const* self__pod, void* destBuffer, uint32_t destBufferSize) {
        physx::PxHeightField const* self_ = reinterpret_cast<physx::PxHeightField const*>(self__pod);
        uint32_t return_val = self_->saveCells(destBuffer, destBufferSize);
        return return_val;
    }

    bool PxHeightField_modifySamples(physx_PxHeightField* self__pod, int32_t startCol, int32_t startRow, physx_PxHeightFieldDesc const* subfieldDesc_pod, bool shrinkBounds) {
        physx::PxHeightField* self_ = reinterpret_cast<physx::PxHeightField*>(self__pod);
        physx::PxHeightFieldDesc const& subfieldDesc = reinterpret_cast<physx::PxHeightFieldDesc const&>(*subfieldDesc_pod);
        bool return_val = self_->modifySamples(startCol, startRow, subfieldDesc, shrinkBounds);
        return return_val;
    }

    uint32_t PxHeightField_getNbRows(physx_PxHeightField const* self__pod) {
        physx::PxHeightField const* self_ = reinterpret_cast<physx::PxHeightField const*>(self__pod);
        uint32_t return_val = self_->getNbRows();
        return return_val;
    }

    uint32_t PxHeightField_getNbColumns(physx_PxHeightField const* self__pod) {
        physx::PxHeightField const* self_ = reinterpret_cast<physx::PxHeightField const*>(self__pod);
        uint32_t return_val = self_->getNbColumns();
        return return_val;
    }

    PxHeightFieldFormat PxHeightField_getFormat(physx_PxHeightField const* self__pod) {
        physx::PxHeightField const* self_ = reinterpret_cast<physx::PxHeightField const*>(self__pod);
        physx::PxHeightFieldFormat::Enum return_val = self_->getFormat();
        PxHeightFieldFormat return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    uint32_t PxHeightField_getSampleStride(physx_PxHeightField const* self__pod) {
        physx::PxHeightField const* self_ = reinterpret_cast<physx::PxHeightField const*>(self__pod);
        uint32_t return_val = self_->getSampleStride();
        return return_val;
    }

    float PxHeightField_getConvexEdgeThreshold(physx_PxHeightField const* self__pod) {
        physx::PxHeightField const* self_ = reinterpret_cast<physx::PxHeightField const*>(self__pod);
        float return_val = self_->getConvexEdgeThreshold();
        return return_val;
    }

    PxHeightFieldFlags PxHeightField_getFlags(physx_PxHeightField const* self__pod) {
        physx::PxHeightField const* self_ = reinterpret_cast<physx::PxHeightField const*>(self__pod);
        physx::PxHeightFieldFlags return_val = self_->getFlags();
        PxHeightFieldFlags return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    float PxHeightField_getHeight(physx_PxHeightField const* self__pod, float x, float z) {
        physx::PxHeightField const* self_ = reinterpret_cast<physx::PxHeightField const*>(self__pod);
        float return_val = self_->getHeight(x, z);
        return return_val;
    }

    uint16_t PxHeightField_getTriangleMaterialIndex(physx_PxHeightField const* self__pod, uint32_t triangleIndex) {
        physx::PxHeightField const* self_ = reinterpret_cast<physx::PxHeightField const*>(self__pod);
        uint16_t return_val = self_->getTriangleMaterialIndex(triangleIndex);
        return return_val;
    }

    physx_PxVec3 PxHeightField_getTriangleNormal(physx_PxHeightField const* self__pod, uint32_t triangleIndex) {
        physx::PxHeightField const* self_ = reinterpret_cast<physx::PxHeightField const*>(self__pod);
        physx::PxVec3 return_val = self_->getTriangleNormal(triangleIndex);
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxHeightFieldSample const* PxHeightField_getSample(physx_PxHeightField const* self__pod, uint32_t row, uint32_t column) {
        physx::PxHeightField const* self_ = reinterpret_cast<physx::PxHeightField const*>(self__pod);
        physx::PxHeightFieldSample const& return_val = self_->getSample(row, column);
        auto return_val_pod = reinterpret_cast<physx_PxHeightFieldSample const*>(&return_val);
        return return_val_pod;
    }

    uint32_t PxHeightField_getTimestamp(physx_PxHeightField const* self__pod) {
        physx::PxHeightField const* self_ = reinterpret_cast<physx::PxHeightField const*>(self__pod);
        uint32_t return_val = self_->getTimestamp();
        return return_val;
    }

    char const* PxHeightField_getConcreteTypeName(physx_PxHeightField const* self__pod) {
        physx::PxHeightField const* self_ = reinterpret_cast<physx::PxHeightField const*>(self__pod);
        char const* return_val = self_->getConcreteTypeName();
        return return_val;
    }

    physx_PxHeightFieldDesc PxHeightFieldDesc_new() {
        PxHeightFieldDesc return_val;
        physx_PxHeightFieldDesc return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxHeightFieldDesc_setToDefault(physx_PxHeightFieldDesc* self__pod) {
        physx::PxHeightFieldDesc* self_ = reinterpret_cast<physx::PxHeightFieldDesc*>(self__pod);
        self_->setToDefault();
    }

    bool PxHeightFieldDesc_isValid(physx_PxHeightFieldDesc const* self__pod) {
        physx::PxHeightFieldDesc const* self_ = reinterpret_cast<physx::PxHeightFieldDesc const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    void PxMeshQuery_getTriangle(physx_PxTriangleMeshGeometry const* triGeom_pod, physx_PxTransform const* transform_pod, uint32_t triangleIndex, physx_PxTriangle* triangle_pod, uint32_t* vertexIndices, uint32_t* adjacencyIndices) {
        physx::PxTriangleMeshGeometry const& triGeom = reinterpret_cast<physx::PxTriangleMeshGeometry const&>(*triGeom_pod);
        physx::PxTransform const& transform = reinterpret_cast<physx::PxTransform const&>(*transform_pod);
        physx::PxTriangle& triangle = reinterpret_cast<physx::PxTriangle&>(*triangle_pod);
        PxMeshQuery::getTriangle(triGeom, transform, triangleIndex, triangle, vertexIndices, adjacencyIndices);
    }

    void PxMeshQuery_getTriangle_1(physx_PxHeightFieldGeometry const* hfGeom_pod, physx_PxTransform const* transform_pod, uint32_t triangleIndex, physx_PxTriangle* triangle_pod, uint32_t* vertexIndices, uint32_t* adjacencyIndices) {
        physx::PxHeightFieldGeometry const& hfGeom = reinterpret_cast<physx::PxHeightFieldGeometry const&>(*hfGeom_pod);
        physx::PxTransform const& transform = reinterpret_cast<physx::PxTransform const&>(*transform_pod);
        physx::PxTriangle& triangle = reinterpret_cast<physx::PxTriangle&>(*triangle_pod);
        PxMeshQuery::getTriangle(hfGeom, transform, triangleIndex, triangle, vertexIndices, adjacencyIndices);
    }

    uint32_t PxMeshQuery_findOverlapTriangleMesh(physx_PxGeometry const* geom_pod, physx_PxTransform const* geomPose_pod, physx_PxTriangleMeshGeometry const* meshGeom_pod, physx_PxTransform const* meshPose_pod, uint32_t* results, uint32_t maxResults, uint32_t startIndex, bool* overflow_pod, PxGeometryQueryFlags queryFlags_pod) {
        physx::PxGeometry const& geom = reinterpret_cast<physx::PxGeometry const&>(*geom_pod);
        physx::PxTransform const& geomPose = reinterpret_cast<physx::PxTransform const&>(*geomPose_pod);
        physx::PxTriangleMeshGeometry const& meshGeom = reinterpret_cast<physx::PxTriangleMeshGeometry const&>(*meshGeom_pod);
        physx::PxTransform const& meshPose = reinterpret_cast<physx::PxTransform const&>(*meshPose_pod);
        bool& overflow = *overflow_pod;
        auto queryFlags = physx::PxGeometryQueryFlags(queryFlags_pod);
        uint32_t return_val = PxMeshQuery::findOverlapTriangleMesh(geom, geomPose, meshGeom, meshPose, results, maxResults, startIndex, overflow, queryFlags);
        return return_val;
    }

    uint32_t PxMeshQuery_findOverlapHeightField(physx_PxGeometry const* geom_pod, physx_PxTransform const* geomPose_pod, physx_PxHeightFieldGeometry const* hfGeom_pod, physx_PxTransform const* hfPose_pod, uint32_t* results, uint32_t maxResults, uint32_t startIndex, bool* overflow_pod, PxGeometryQueryFlags queryFlags_pod) {
        physx::PxGeometry const& geom = reinterpret_cast<physx::PxGeometry const&>(*geom_pod);
        physx::PxTransform const& geomPose = reinterpret_cast<physx::PxTransform const&>(*geomPose_pod);
        physx::PxHeightFieldGeometry const& hfGeom = reinterpret_cast<physx::PxHeightFieldGeometry const&>(*hfGeom_pod);
        physx::PxTransform const& hfPose = reinterpret_cast<physx::PxTransform const&>(*hfPose_pod);
        bool& overflow = *overflow_pod;
        auto queryFlags = physx::PxGeometryQueryFlags(queryFlags_pod);
        uint32_t return_val = PxMeshQuery::findOverlapHeightField(geom, geomPose, hfGeom, hfPose, results, maxResults, startIndex, overflow, queryFlags);
        return return_val;
    }

    bool PxMeshQuery_sweep(physx_PxVec3 const* unitDir_pod, float distance, physx_PxGeometry const* geom_pod, physx_PxTransform const* pose_pod, uint32_t triangleCount, physx_PxTriangle const* triangles_pod, physx_PxGeomSweepHit* sweepHit_pod, PxHitFlags hitFlags_pod, uint32_t const* cachedIndex, float inflation, bool doubleSided, PxGeometryQueryFlags queryFlags_pod) {
        physx::PxVec3 const& unitDir = reinterpret_cast<physx::PxVec3 const&>(*unitDir_pod);
        physx::PxGeometry const& geom = reinterpret_cast<physx::PxGeometry const&>(*geom_pod);
        physx::PxTransform const& pose = reinterpret_cast<physx::PxTransform const&>(*pose_pod);
        physx::PxTriangle const* triangles = reinterpret_cast<physx::PxTriangle const*>(triangles_pod);
        physx::PxGeomSweepHit& sweepHit = reinterpret_cast<physx::PxGeomSweepHit&>(*sweepHit_pod);
        auto hitFlags = physx::PxHitFlags(hitFlags_pod);
        auto queryFlags = physx::PxGeometryQueryFlags(queryFlags_pod);
        bool return_val = PxMeshQuery::sweep(unitDir, distance, geom, pose, triangleCount, triangles, sweepHit, hitFlags, cachedIndex, inflation, doubleSided, queryFlags);
        return return_val;
    }

    physx_PxSimpleTriangleMesh PxSimpleTriangleMesh_new() {
        PxSimpleTriangleMesh return_val;
        physx_PxSimpleTriangleMesh return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxSimpleTriangleMesh_setToDefault(physx_PxSimpleTriangleMesh* self__pod) {
        physx::PxSimpleTriangleMesh* self_ = reinterpret_cast<physx::PxSimpleTriangleMesh*>(self__pod);
        self_->setToDefault();
    }

    bool PxSimpleTriangleMesh_isValid(physx_PxSimpleTriangleMesh const* self__pod) {
        physx::PxSimpleTriangleMesh const* self_ = reinterpret_cast<physx::PxSimpleTriangleMesh const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    physx_PxTriangle* PxTriangle_new_alloc() {
        auto return_val = new physx::PxTriangle();
        auto return_val_pod = reinterpret_cast<physx_PxTriangle*>(return_val);
        return return_val_pod;
    }

    physx_PxTriangle* PxTriangle_new_alloc_1(physx_PxVec3 const* p0_pod, physx_PxVec3 const* p1_pod, physx_PxVec3 const* p2_pod) {
        physx::PxVec3 const& p0 = reinterpret_cast<physx::PxVec3 const&>(*p0_pod);
        physx::PxVec3 const& p1 = reinterpret_cast<physx::PxVec3 const&>(*p1_pod);
        physx::PxVec3 const& p2 = reinterpret_cast<physx::PxVec3 const&>(*p2_pod);
        auto return_val = new physx::PxTriangle(p0, p1, p2);
        auto return_val_pod = reinterpret_cast<physx_PxTriangle*>(return_val);
        return return_val_pod;
    }

    void PxTriangle_delete(physx_PxTriangle* self__pod) {
        physx::PxTriangle* self_ = reinterpret_cast<physx::PxTriangle*>(self__pod);
        delete self_;
    }

    void PxTriangle_normal(physx_PxTriangle const* self__pod, physx_PxVec3* _normal_pod) {
        physx::PxTriangle const* self_ = reinterpret_cast<physx::PxTriangle const*>(self__pod);
        physx::PxVec3& _normal = reinterpret_cast<physx::PxVec3&>(*_normal_pod);
        self_->normal(_normal);
    }

    void PxTriangle_denormalizedNormal(physx_PxTriangle const* self__pod, physx_PxVec3* _normal_pod) {
        physx::PxTriangle const* self_ = reinterpret_cast<physx::PxTriangle const*>(self__pod);
        physx::PxVec3& _normal = reinterpret_cast<physx::PxVec3&>(*_normal_pod);
        self_->denormalizedNormal(_normal);
    }

    float PxTriangle_area(physx_PxTriangle const* self__pod) {
        physx::PxTriangle const* self_ = reinterpret_cast<physx::PxTriangle const*>(self__pod);
        float return_val = self_->area();
        return return_val;
    }

    physx_PxVec3 PxTriangle_pointFromUV(physx_PxTriangle const* self__pod, float u, float v) {
        physx::PxTriangle const* self_ = reinterpret_cast<physx::PxTriangle const*>(self__pod);
        physx::PxVec3 return_val = self_->pointFromUV(u, v);
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxTrianglePadded* PxTrianglePadded_new_alloc() {
        auto return_val = new physx::PxTrianglePadded();
        auto return_val_pod = reinterpret_cast<physx_PxTrianglePadded*>(return_val);
        return return_val_pod;
    }

    void PxTrianglePadded_delete(physx_PxTrianglePadded* self__pod) {
        physx::PxTrianglePadded* self_ = reinterpret_cast<physx::PxTrianglePadded*>(self__pod);
        delete self_;
    }

    uint32_t PxTriangleMesh_getNbVertices(physx_PxTriangleMesh const* self__pod) {
        physx::PxTriangleMesh const* self_ = reinterpret_cast<physx::PxTriangleMesh const*>(self__pod);
        uint32_t return_val = self_->getNbVertices();
        return return_val;
    }

    physx_PxVec3 const* PxTriangleMesh_getVertices(physx_PxTriangleMesh const* self__pod) {
        physx::PxTriangleMesh const* self_ = reinterpret_cast<physx::PxTriangleMesh const*>(self__pod);
        physx::PxVec3 const* return_val = self_->getVertices();
        auto return_val_pod = reinterpret_cast<physx_PxVec3 const*>(return_val);
        return return_val_pod;
    }

    physx_PxVec3* PxTriangleMesh_getVerticesForModification(physx_PxTriangleMesh* self__pod) {
        physx::PxTriangleMesh* self_ = reinterpret_cast<physx::PxTriangleMesh*>(self__pod);
        physx::PxVec3* return_val = self_->getVerticesForModification();
        auto return_val_pod = reinterpret_cast<physx_PxVec3*>(return_val);
        return return_val_pod;
    }

    physx_PxBounds3 PxTriangleMesh_refitBVH(physx_PxTriangleMesh* self__pod) {
        physx::PxTriangleMesh* self_ = reinterpret_cast<physx::PxTriangleMesh*>(self__pod);
        physx::PxBounds3 return_val = self_->refitBVH();
        physx_PxBounds3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    uint32_t PxTriangleMesh_getNbTriangles(physx_PxTriangleMesh const* self__pod) {
        physx::PxTriangleMesh const* self_ = reinterpret_cast<physx::PxTriangleMesh const*>(self__pod);
        uint32_t return_val = self_->getNbTriangles();
        return return_val;
    }

    void const* PxTriangleMesh_getTriangles(physx_PxTriangleMesh const* self__pod) {
        physx::PxTriangleMesh const* self_ = reinterpret_cast<physx::PxTriangleMesh const*>(self__pod);
        void const* return_val = self_->getTriangles();
        return return_val;
    }

    PxTriangleMeshFlags PxTriangleMesh_getTriangleMeshFlags(physx_PxTriangleMesh const* self__pod) {
        physx::PxTriangleMesh const* self_ = reinterpret_cast<physx::PxTriangleMesh const*>(self__pod);
        physx::PxTriangleMeshFlags return_val = self_->getTriangleMeshFlags();
        PxTriangleMeshFlags return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    uint32_t const* PxTriangleMesh_getTrianglesRemap(physx_PxTriangleMesh const* self__pod) {
        physx::PxTriangleMesh const* self_ = reinterpret_cast<physx::PxTriangleMesh const*>(self__pod);
        uint32_t const* return_val = self_->getTrianglesRemap();
        return return_val;
    }

    void PxTriangleMesh_release(physx_PxTriangleMesh* self__pod) {
        physx::PxTriangleMesh* self_ = reinterpret_cast<physx::PxTriangleMesh*>(self__pod);
        self_->release();
    }

    uint16_t PxTriangleMesh_getTriangleMaterialIndex(physx_PxTriangleMesh const* self__pod, uint32_t triangleIndex) {
        physx::PxTriangleMesh const* self_ = reinterpret_cast<physx::PxTriangleMesh const*>(self__pod);
        uint16_t return_val = self_->getTriangleMaterialIndex(triangleIndex);
        return return_val;
    }

    physx_PxBounds3 PxTriangleMesh_getLocalBounds(physx_PxTriangleMesh const* self__pod) {
        physx::PxTriangleMesh const* self_ = reinterpret_cast<physx::PxTriangleMesh const*>(self__pod);
        physx::PxBounds3 return_val = self_->getLocalBounds();
        physx_PxBounds3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    float const* PxTriangleMesh_getSDF(physx_PxTriangleMesh const* self__pod) {
        physx::PxTriangleMesh const* self_ = reinterpret_cast<physx::PxTriangleMesh const*>(self__pod);
        float const* return_val = self_->getSDF();
        return return_val;
    }

    void PxTriangleMesh_getSDFDimensions(physx_PxTriangleMesh const* self__pod, uint32_t* numX_pod, uint32_t* numY_pod, uint32_t* numZ_pod) {
        physx::PxTriangleMesh const* self_ = reinterpret_cast<physx::PxTriangleMesh const*>(self__pod);
        uint32_t& numX = *numX_pod;
        uint32_t& numY = *numY_pod;
        uint32_t& numZ = *numZ_pod;
        self_->getSDFDimensions(numX, numY, numZ);
    }

    void PxTriangleMesh_setPreferSDFProjection(physx_PxTriangleMesh* self__pod, bool preferProjection) {
        physx::PxTriangleMesh* self_ = reinterpret_cast<physx::PxTriangleMesh*>(self__pod);
        self_->setPreferSDFProjection(preferProjection);
    }

    bool PxTriangleMesh_getPreferSDFProjection(physx_PxTriangleMesh const* self__pod) {
        physx::PxTriangleMesh const* self_ = reinterpret_cast<physx::PxTriangleMesh const*>(self__pod);
        bool return_val = self_->getPreferSDFProjection();
        return return_val;
    }

    void PxTriangleMesh_getMassInformation(physx_PxTriangleMesh const* self__pod, float* mass_pod, physx_PxMat33* localInertia_pod, physx_PxVec3* localCenterOfMass_pod) {
        physx::PxTriangleMesh const* self_ = reinterpret_cast<physx::PxTriangleMesh const*>(self__pod);
        float& mass = *mass_pod;
        physx::PxMat33& localInertia = reinterpret_cast<physx::PxMat33&>(*localInertia_pod);
        physx::PxVec3& localCenterOfMass = reinterpret_cast<physx::PxVec3&>(*localCenterOfMass_pod);
        self_->getMassInformation(mass, localInertia, localCenterOfMass);
    }

    physx_PxTetrahedron* PxTetrahedron_new_alloc() {
        auto return_val = new physx::PxTetrahedron();
        auto return_val_pod = reinterpret_cast<physx_PxTetrahedron*>(return_val);
        return return_val_pod;
    }

    physx_PxTetrahedron* PxTetrahedron_new_alloc_1(physx_PxVec3 const* p0_pod, physx_PxVec3 const* p1_pod, physx_PxVec3 const* p2_pod, physx_PxVec3 const* p3_pod) {
        physx::PxVec3 const& p0 = reinterpret_cast<physx::PxVec3 const&>(*p0_pod);
        physx::PxVec3 const& p1 = reinterpret_cast<physx::PxVec3 const&>(*p1_pod);
        physx::PxVec3 const& p2 = reinterpret_cast<physx::PxVec3 const&>(*p2_pod);
        physx::PxVec3 const& p3 = reinterpret_cast<physx::PxVec3 const&>(*p3_pod);
        auto return_val = new physx::PxTetrahedron(p0, p1, p2, p3);
        auto return_val_pod = reinterpret_cast<physx_PxTetrahedron*>(return_val);
        return return_val_pod;
    }

    void PxTetrahedron_delete(physx_PxTetrahedron* self__pod) {
        physx::PxTetrahedron* self_ = reinterpret_cast<physx::PxTetrahedron*>(self__pod);
        delete self_;
    }

    void PxSoftBodyAuxData_release(physx_PxSoftBodyAuxData* self__pod) {
        physx::PxSoftBodyAuxData* self_ = reinterpret_cast<physx::PxSoftBodyAuxData*>(self__pod);
        self_->release();
    }

    uint32_t PxTetrahedronMesh_getNbVertices(physx_PxTetrahedronMesh const* self__pod) {
        physx::PxTetrahedronMesh const* self_ = reinterpret_cast<physx::PxTetrahedronMesh const*>(self__pod);
        uint32_t return_val = self_->getNbVertices();
        return return_val;
    }

    physx_PxVec3 const* PxTetrahedronMesh_getVertices(physx_PxTetrahedronMesh const* self__pod) {
        physx::PxTetrahedronMesh const* self_ = reinterpret_cast<physx::PxTetrahedronMesh const*>(self__pod);
        physx::PxVec3 const* return_val = self_->getVertices();
        auto return_val_pod = reinterpret_cast<physx_PxVec3 const*>(return_val);
        return return_val_pod;
    }

    uint32_t PxTetrahedronMesh_getNbTetrahedrons(physx_PxTetrahedronMesh const* self__pod) {
        physx::PxTetrahedronMesh const* self_ = reinterpret_cast<physx::PxTetrahedronMesh const*>(self__pod);
        uint32_t return_val = self_->getNbTetrahedrons();
        return return_val;
    }

    void const* PxTetrahedronMesh_getTetrahedrons(physx_PxTetrahedronMesh const* self__pod) {
        physx::PxTetrahedronMesh const* self_ = reinterpret_cast<physx::PxTetrahedronMesh const*>(self__pod);
        void const* return_val = self_->getTetrahedrons();
        return return_val;
    }

    PxTetrahedronMeshFlags PxTetrahedronMesh_getTetrahedronMeshFlags(physx_PxTetrahedronMesh const* self__pod) {
        physx::PxTetrahedronMesh const* self_ = reinterpret_cast<physx::PxTetrahedronMesh const*>(self__pod);
        physx::PxTetrahedronMeshFlags return_val = self_->getTetrahedronMeshFlags();
        PxTetrahedronMeshFlags return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    uint32_t const* PxTetrahedronMesh_getTetrahedraRemap(physx_PxTetrahedronMesh const* self__pod) {
        physx::PxTetrahedronMesh const* self_ = reinterpret_cast<physx::PxTetrahedronMesh const*>(self__pod);
        uint32_t const* return_val = self_->getTetrahedraRemap();
        return return_val;
    }

    physx_PxBounds3 PxTetrahedronMesh_getLocalBounds(physx_PxTetrahedronMesh const* self__pod) {
        physx::PxTetrahedronMesh const* self_ = reinterpret_cast<physx::PxTetrahedronMesh const*>(self__pod);
        physx::PxBounds3 return_val = self_->getLocalBounds();
        physx_PxBounds3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxTetrahedronMesh_release(physx_PxTetrahedronMesh* self__pod) {
        physx::PxTetrahedronMesh* self_ = reinterpret_cast<physx::PxTetrahedronMesh*>(self__pod);
        self_->release();
    }

    physx_PxTetrahedronMesh const* PxSoftBodyMesh_getCollisionMesh(physx_PxSoftBodyMesh const* self__pod) {
        physx::PxSoftBodyMesh const* self_ = reinterpret_cast<physx::PxSoftBodyMesh const*>(self__pod);
        physx::PxTetrahedronMesh const* return_val = self_->getCollisionMesh();
        auto return_val_pod = reinterpret_cast<physx_PxTetrahedronMesh const*>(return_val);
        return return_val_pod;
    }

    physx_PxTetrahedronMesh* PxSoftBodyMesh_getCollisionMesh_1(physx_PxSoftBodyMesh* self__pod) {
        physx::PxSoftBodyMesh* self_ = reinterpret_cast<physx::PxSoftBodyMesh*>(self__pod);
        physx::PxTetrahedronMesh* return_val = self_->getCollisionMesh();
        auto return_val_pod = reinterpret_cast<physx_PxTetrahedronMesh*>(return_val);
        return return_val_pod;
    }

    physx_PxTetrahedronMesh const* PxSoftBodyMesh_getSimulationMesh(physx_PxSoftBodyMesh const* self__pod) {
        physx::PxSoftBodyMesh const* self_ = reinterpret_cast<physx::PxSoftBodyMesh const*>(self__pod);
        physx::PxTetrahedronMesh const* return_val = self_->getSimulationMesh();
        auto return_val_pod = reinterpret_cast<physx_PxTetrahedronMesh const*>(return_val);
        return return_val_pod;
    }

    physx_PxTetrahedronMesh* PxSoftBodyMesh_getSimulationMesh_1(physx_PxSoftBodyMesh* self__pod) {
        physx::PxSoftBodyMesh* self_ = reinterpret_cast<physx::PxSoftBodyMesh*>(self__pod);
        physx::PxTetrahedronMesh* return_val = self_->getSimulationMesh();
        auto return_val_pod = reinterpret_cast<physx_PxTetrahedronMesh*>(return_val);
        return return_val_pod;
    }

    physx_PxSoftBodyAuxData const* PxSoftBodyMesh_getSoftBodyAuxData(physx_PxSoftBodyMesh const* self__pod) {
        physx::PxSoftBodyMesh const* self_ = reinterpret_cast<physx::PxSoftBodyMesh const*>(self__pod);
        physx::PxSoftBodyAuxData const* return_val = self_->getSoftBodyAuxData();
        auto return_val_pod = reinterpret_cast<physx_PxSoftBodyAuxData const*>(return_val);
        return return_val_pod;
    }

    physx_PxSoftBodyAuxData* PxSoftBodyMesh_getSoftBodyAuxData_1(physx_PxSoftBodyMesh* self__pod) {
        physx::PxSoftBodyMesh* self_ = reinterpret_cast<physx::PxSoftBodyMesh*>(self__pod);
        physx::PxSoftBodyAuxData* return_val = self_->getSoftBodyAuxData();
        auto return_val_pod = reinterpret_cast<physx_PxSoftBodyAuxData*>(return_val);
        return return_val_pod;
    }

    void PxSoftBodyMesh_release(physx_PxSoftBodyMesh* self__pod) {
        physx::PxSoftBodyMesh* self_ = reinterpret_cast<physx::PxSoftBodyMesh*>(self__pod);
        self_->release();
    }

    void PxCollisionMeshMappingData_release(physx_PxCollisionMeshMappingData* self__pod) {
        physx::PxCollisionMeshMappingData* self_ = reinterpret_cast<physx::PxCollisionMeshMappingData*>(self__pod);
        self_->release();
    }

    physx_PxTetrahedronMeshData const* PxCollisionTetrahedronMeshData_getMesh(physx_PxCollisionTetrahedronMeshData const* self__pod) {
        physx::PxCollisionTetrahedronMeshData const* self_ = reinterpret_cast<physx::PxCollisionTetrahedronMeshData const*>(self__pod);
        physx::PxTetrahedronMeshData const* return_val = self_->getMesh();
        auto return_val_pod = reinterpret_cast<physx_PxTetrahedronMeshData const*>(return_val);
        return return_val_pod;
    }

    physx_PxTetrahedronMeshData* PxCollisionTetrahedronMeshData_getMesh_1(physx_PxCollisionTetrahedronMeshData* self__pod) {
        physx::PxCollisionTetrahedronMeshData* self_ = reinterpret_cast<physx::PxCollisionTetrahedronMeshData*>(self__pod);
        physx::PxTetrahedronMeshData* return_val = self_->getMesh();
        auto return_val_pod = reinterpret_cast<physx_PxTetrahedronMeshData*>(return_val);
        return return_val_pod;
    }

    physx_PxSoftBodyCollisionData const* PxCollisionTetrahedronMeshData_getData(physx_PxCollisionTetrahedronMeshData const* self__pod) {
        physx::PxCollisionTetrahedronMeshData const* self_ = reinterpret_cast<physx::PxCollisionTetrahedronMeshData const*>(self__pod);
        physx::PxSoftBodyCollisionData const* return_val = self_->getData();
        auto return_val_pod = reinterpret_cast<physx_PxSoftBodyCollisionData const*>(return_val);
        return return_val_pod;
    }

    physx_PxSoftBodyCollisionData* PxCollisionTetrahedronMeshData_getData_1(physx_PxCollisionTetrahedronMeshData* self__pod) {
        physx::PxCollisionTetrahedronMeshData* self_ = reinterpret_cast<physx::PxCollisionTetrahedronMeshData*>(self__pod);
        physx::PxSoftBodyCollisionData* return_val = self_->getData();
        auto return_val_pod = reinterpret_cast<physx_PxSoftBodyCollisionData*>(return_val);
        return return_val_pod;
    }

    void PxCollisionTetrahedronMeshData_release(physx_PxCollisionTetrahedronMeshData* self__pod) {
        physx::PxCollisionTetrahedronMeshData* self_ = reinterpret_cast<physx::PxCollisionTetrahedronMeshData*>(self__pod);
        self_->release();
    }

    physx_PxTetrahedronMeshData* PxSimulationTetrahedronMeshData_getMesh(physx_PxSimulationTetrahedronMeshData* self__pod) {
        physx::PxSimulationTetrahedronMeshData* self_ = reinterpret_cast<physx::PxSimulationTetrahedronMeshData*>(self__pod);
        physx::PxTetrahedronMeshData* return_val = self_->getMesh();
        auto return_val_pod = reinterpret_cast<physx_PxTetrahedronMeshData*>(return_val);
        return return_val_pod;
    }

    physx_PxSoftBodySimulationData* PxSimulationTetrahedronMeshData_getData(physx_PxSimulationTetrahedronMeshData* self__pod) {
        physx::PxSimulationTetrahedronMeshData* self_ = reinterpret_cast<physx::PxSimulationTetrahedronMeshData*>(self__pod);
        physx::PxSoftBodySimulationData* return_val = self_->getData();
        auto return_val_pod = reinterpret_cast<physx_PxSoftBodySimulationData*>(return_val);
        return return_val_pod;
    }

    void PxSimulationTetrahedronMeshData_release(physx_PxSimulationTetrahedronMeshData* self__pod) {
        physx::PxSimulationTetrahedronMeshData* self_ = reinterpret_cast<physx::PxSimulationTetrahedronMeshData*>(self__pod);
        self_->release();
    }

    void PxActor_release(physx_PxActor* self__pod) {
        physx::PxActor* self_ = reinterpret_cast<physx::PxActor*>(self__pod);
        self_->release();
    }

    PxActorType PxActor_getType(physx_PxActor const* self__pod) {
        physx::PxActor const* self_ = reinterpret_cast<physx::PxActor const*>(self__pod);
        physx::PxActorType::Enum return_val = self_->getType();
        PxActorType return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxScene* PxActor_getScene(physx_PxActor const* self__pod) {
        physx::PxActor const* self_ = reinterpret_cast<physx::PxActor const*>(self__pod);
        physx::PxScene* return_val = self_->getScene();
        auto return_val_pod = reinterpret_cast<physx_PxScene*>(return_val);
        return return_val_pod;
    }

    void PxActor_setName(physx_PxActor* self__pod, char const* name) {
        physx::PxActor* self_ = reinterpret_cast<physx::PxActor*>(self__pod);
        self_->setName(name);
    }

    char const* PxActor_getName(physx_PxActor const* self__pod) {
        physx::PxActor const* self_ = reinterpret_cast<physx::PxActor const*>(self__pod);
        char const* return_val = self_->getName();
        return return_val;
    }

    physx_PxBounds3 PxActor_getWorldBounds(physx_PxActor const* self__pod, float inflation) {
        physx::PxActor const* self_ = reinterpret_cast<physx::PxActor const*>(self__pod);
        physx::PxBounds3 return_val = self_->getWorldBounds(inflation);
        physx_PxBounds3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxActor_setActorFlag(physx_PxActor* self__pod, PxActorFlag flag_pod, bool value) {
        physx::PxActor* self_ = reinterpret_cast<physx::PxActor*>(self__pod);
        auto flag = static_cast<physx::PxActorFlag::Enum>(flag_pod);
        self_->setActorFlag(flag, value);
    }

    void PxActor_setActorFlags(physx_PxActor* self__pod, PxActorFlags inFlags_pod) {
        physx::PxActor* self_ = reinterpret_cast<physx::PxActor*>(self__pod);
        auto inFlags = physx::PxActorFlags(inFlags_pod);
        self_->setActorFlags(inFlags);
    }

    PxActorFlags PxActor_getActorFlags(physx_PxActor const* self__pod) {
        physx::PxActor const* self_ = reinterpret_cast<physx::PxActor const*>(self__pod);
        physx::PxActorFlags return_val = self_->getActorFlags();
        PxActorFlags return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxActor_setDominanceGroup(physx_PxActor* self__pod, uint8_t dominanceGroup) {
        physx::PxActor* self_ = reinterpret_cast<physx::PxActor*>(self__pod);
        self_->setDominanceGroup(dominanceGroup);
    }

    uint8_t PxActor_getDominanceGroup(physx_PxActor const* self__pod) {
        physx::PxActor const* self_ = reinterpret_cast<physx::PxActor const*>(self__pod);
        uint8_t return_val = self_->getDominanceGroup();
        return return_val;
    }

    void PxActor_setOwnerClient(physx_PxActor* self__pod, uint8_t inClient) {
        physx::PxActor* self_ = reinterpret_cast<physx::PxActor*>(self__pod);
        self_->setOwnerClient(inClient);
    }

    uint8_t PxActor_getOwnerClient(physx_PxActor const* self__pod) {
        physx::PxActor const* self_ = reinterpret_cast<physx::PxActor const*>(self__pod);
        uint8_t return_val = self_->getOwnerClient();
        return return_val;
    }

    physx_PxAggregate* PxActor_getAggregate(physx_PxActor const* self__pod) {
        physx::PxActor const* self_ = reinterpret_cast<physx::PxActor const*>(self__pod);
        physx::PxAggregate* return_val = self_->getAggregate();
        auto return_val_pod = reinterpret_cast<physx_PxAggregate*>(return_val);
        return return_val_pod;
    }

    uint32_t phys_PxGetAggregateFilterHint(PxAggregateType type_pod, bool enableSelfCollision) {
        auto type = static_cast<physx::PxAggregateType::Enum>(type_pod);
        uint32_t return_val = PxGetAggregateFilterHint(type, enableSelfCollision);
        return return_val;
    }

    uint32_t phys_PxGetAggregateSelfCollisionBit(uint32_t hint) {
        uint32_t return_val = PxGetAggregateSelfCollisionBit(hint);
        return return_val;
    }

    PxAggregateType phys_PxGetAggregateType(uint32_t hint) {
        physx::PxAggregateType::Enum return_val = PxGetAggregateType(hint);
        PxAggregateType return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxAggregate_release(physx_PxAggregate* self__pod) {
        physx::PxAggregate* self_ = reinterpret_cast<physx::PxAggregate*>(self__pod);
        self_->release();
    }

    bool PxAggregate_addActor(physx_PxAggregate* self__pod, physx_PxActor* actor_pod, physx_PxBVH const* bvh_pod) {
        physx::PxAggregate* self_ = reinterpret_cast<physx::PxAggregate*>(self__pod);
        physx::PxActor& actor = reinterpret_cast<physx::PxActor&>(*actor_pod);
        physx::PxBVH const* bvh = reinterpret_cast<physx::PxBVH const*>(bvh_pod);
        bool return_val = self_->addActor(actor, bvh);
        return return_val;
    }

    bool PxAggregate_removeActor(physx_PxAggregate* self__pod, physx_PxActor* actor_pod) {
        physx::PxAggregate* self_ = reinterpret_cast<physx::PxAggregate*>(self__pod);
        physx::PxActor& actor = reinterpret_cast<physx::PxActor&>(*actor_pod);
        bool return_val = self_->removeActor(actor);
        return return_val;
    }

    bool PxAggregate_addArticulation(physx_PxAggregate* self__pod, physx_PxArticulationReducedCoordinate* articulation_pod) {
        physx::PxAggregate* self_ = reinterpret_cast<physx::PxAggregate*>(self__pod);
        physx::PxArticulationReducedCoordinate& articulation = reinterpret_cast<physx::PxArticulationReducedCoordinate&>(*articulation_pod);
        bool return_val = self_->addArticulation(articulation);
        return return_val;
    }

    bool PxAggregate_removeArticulation(physx_PxAggregate* self__pod, physx_PxArticulationReducedCoordinate* articulation_pod) {
        physx::PxAggregate* self_ = reinterpret_cast<physx::PxAggregate*>(self__pod);
        physx::PxArticulationReducedCoordinate& articulation = reinterpret_cast<physx::PxArticulationReducedCoordinate&>(*articulation_pod);
        bool return_val = self_->removeArticulation(articulation);
        return return_val;
    }

    uint32_t PxAggregate_getNbActors(physx_PxAggregate const* self__pod) {
        physx::PxAggregate const* self_ = reinterpret_cast<physx::PxAggregate const*>(self__pod);
        uint32_t return_val = self_->getNbActors();
        return return_val;
    }

    uint32_t PxAggregate_getMaxNbShapes(physx_PxAggregate const* self__pod) {
        physx::PxAggregate const* self_ = reinterpret_cast<physx::PxAggregate const*>(self__pod);
        uint32_t return_val = self_->getMaxNbShapes();
        return return_val;
    }

    uint32_t PxAggregate_getActors(physx_PxAggregate const* self__pod, physx_PxActor** userBuffer_pod, uint32_t bufferSize, uint32_t startIndex) {
        physx::PxAggregate const* self_ = reinterpret_cast<physx::PxAggregate const*>(self__pod);
        physx::PxActor** userBuffer = reinterpret_cast<physx::PxActor**>(userBuffer_pod);
        uint32_t return_val = self_->getActors(userBuffer, bufferSize, startIndex);
        return return_val;
    }

    physx_PxScene* PxAggregate_getScene(physx_PxAggregate* self__pod) {
        physx::PxAggregate* self_ = reinterpret_cast<physx::PxAggregate*>(self__pod);
        physx::PxScene* return_val = self_->getScene();
        auto return_val_pod = reinterpret_cast<physx_PxScene*>(return_val);
        return return_val_pod;
    }

    bool PxAggregate_getSelfCollision(physx_PxAggregate const* self__pod) {
        physx::PxAggregate const* self_ = reinterpret_cast<physx::PxAggregate const*>(self__pod);
        bool return_val = self_->getSelfCollision();
        return return_val;
    }

    char const* PxAggregate_getConcreteTypeName(physx_PxAggregate const* self__pod) {
        physx::PxAggregate const* self_ = reinterpret_cast<physx::PxAggregate const*>(self__pod);
        char const* return_val = self_->getConcreteTypeName();
        return return_val;
    }

    physx_PxConstraintInvMassScale PxConstraintInvMassScale_new() {
        PxConstraintInvMassScale return_val;
        physx_PxConstraintInvMassScale return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxConstraintInvMassScale PxConstraintInvMassScale_new_1(float lin0, float ang0, float lin1, float ang1) {
        PxConstraintInvMassScale return_val(lin0, ang0, lin1, ang1);
        physx_PxConstraintInvMassScale return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxConstraintVisualizer_visualizeJointFrames(physx_PxConstraintVisualizer* self__pod, physx_PxTransform const* parent_pod, physx_PxTransform const* child_pod) {
        physx::PxConstraintVisualizer* self_ = reinterpret_cast<physx::PxConstraintVisualizer*>(self__pod);
        physx::PxTransform const& parent = reinterpret_cast<physx::PxTransform const&>(*parent_pod);
        physx::PxTransform const& child = reinterpret_cast<physx::PxTransform const&>(*child_pod);
        self_->visualizeJointFrames(parent, child);
    }

    void PxConstraintVisualizer_visualizeLinearLimit(physx_PxConstraintVisualizer* self__pod, physx_PxTransform const* t0_pod, physx_PxTransform const* t1_pod, float value, bool active) {
        physx::PxConstraintVisualizer* self_ = reinterpret_cast<physx::PxConstraintVisualizer*>(self__pod);
        physx::PxTransform const& t0 = reinterpret_cast<physx::PxTransform const&>(*t0_pod);
        physx::PxTransform const& t1 = reinterpret_cast<physx::PxTransform const&>(*t1_pod);
        self_->visualizeLinearLimit(t0, t1, value, active);
    }

    void PxConstraintVisualizer_visualizeAngularLimit(physx_PxConstraintVisualizer* self__pod, physx_PxTransform const* t0_pod, float lower, float upper, bool active) {
        physx::PxConstraintVisualizer* self_ = reinterpret_cast<physx::PxConstraintVisualizer*>(self__pod);
        physx::PxTransform const& t0 = reinterpret_cast<physx::PxTransform const&>(*t0_pod);
        self_->visualizeAngularLimit(t0, lower, upper, active);
    }

    void PxConstraintVisualizer_visualizeLimitCone(physx_PxConstraintVisualizer* self__pod, physx_PxTransform const* t_pod, float tanQSwingY, float tanQSwingZ, bool active) {
        physx::PxConstraintVisualizer* self_ = reinterpret_cast<physx::PxConstraintVisualizer*>(self__pod);
        physx::PxTransform const& t = reinterpret_cast<physx::PxTransform const&>(*t_pod);
        self_->visualizeLimitCone(t, tanQSwingY, tanQSwingZ, active);
    }

    void PxConstraintVisualizer_visualizeDoubleCone(physx_PxConstraintVisualizer* self__pod, physx_PxTransform const* t_pod, float angle, bool active) {
        physx::PxConstraintVisualizer* self_ = reinterpret_cast<physx::PxConstraintVisualizer*>(self__pod);
        physx::PxTransform const& t = reinterpret_cast<physx::PxTransform const&>(*t_pod);
        self_->visualizeDoubleCone(t, angle, active);
    }

    void PxConstraintVisualizer_visualizeLine(physx_PxConstraintVisualizer* self__pod, physx_PxVec3 const* p0_pod, physx_PxVec3 const* p1_pod, uint32_t color) {
        physx::PxConstraintVisualizer* self_ = reinterpret_cast<physx::PxConstraintVisualizer*>(self__pod);
        physx::PxVec3 const& p0 = reinterpret_cast<physx::PxVec3 const&>(*p0_pod);
        physx::PxVec3 const& p1 = reinterpret_cast<physx::PxVec3 const&>(*p1_pod);
        self_->visualizeLine(p0, p1, color);
    }

    void* PxConstraintConnector_prepareData(physx_PxConstraintConnector* self__pod) {
        physx::PxConstraintConnector* self_ = reinterpret_cast<physx::PxConstraintConnector*>(self__pod);
        void* return_val = self_->prepareData();
        return return_val;
    }

    void PxConstraintConnector_onConstraintRelease(physx_PxConstraintConnector* self__pod) {
        physx::PxConstraintConnector* self_ = reinterpret_cast<physx::PxConstraintConnector*>(self__pod);
        self_->onConstraintRelease();
    }

    void PxConstraintConnector_onComShift(physx_PxConstraintConnector* self__pod, uint32_t actor) {
        physx::PxConstraintConnector* self_ = reinterpret_cast<physx::PxConstraintConnector*>(self__pod);
        self_->onComShift(actor);
    }

    void PxConstraintConnector_onOriginShift(physx_PxConstraintConnector* self__pod, physx_PxVec3 const* shift_pod) {
        physx::PxConstraintConnector* self_ = reinterpret_cast<physx::PxConstraintConnector*>(self__pod);
        physx::PxVec3 const& shift = reinterpret_cast<physx::PxVec3 const&>(*shift_pod);
        self_->onOriginShift(shift);
    }

    physx_PxBase* PxConstraintConnector_getSerializable(physx_PxConstraintConnector* self__pod) {
        physx::PxConstraintConnector* self_ = reinterpret_cast<physx::PxConstraintConnector*>(self__pod);
        physx::PxBase* return_val = self_->getSerializable();
        auto return_val_pod = reinterpret_cast<physx_PxBase*>(return_val);
        return return_val_pod;
    }

    void const* PxConstraintConnector_getConstantBlock(physx_PxConstraintConnector const* self__pod) {
        physx::PxConstraintConnector const* self_ = reinterpret_cast<physx::PxConstraintConnector const*>(self__pod);
        void const* return_val = self_->getConstantBlock();
        return return_val;
    }

    void PxConstraintConnector_connectToConstraint(physx_PxConstraintConnector* self__pod, physx_PxConstraint* anon_param0_pod) {
        physx::PxConstraintConnector* self_ = reinterpret_cast<physx::PxConstraintConnector*>(self__pod);
        physx::PxConstraint* anon_param0 = reinterpret_cast<physx::PxConstraint*>(anon_param0_pod);
        self_->connectToConstraint(anon_param0);
    }

    void PxConstraintConnector_delete(physx_PxConstraintConnector* self__pod) {
        physx::PxConstraintConnector* self_ = reinterpret_cast<physx::PxConstraintConnector*>(self__pod);
        delete self_;
    }

    physx_PxSolverBody PxSolverBody_new() {
        PxSolverBody return_val;
        physx_PxSolverBody return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    float PxSolverBodyData_projectVelocity(physx_PxSolverBodyData const* self__pod, physx_PxVec3 const* lin_pod, physx_PxVec3 const* ang_pod) {
        physx::PxSolverBodyData const* self_ = reinterpret_cast<physx::PxSolverBodyData const*>(self__pod);
        physx::PxVec3 const& lin = reinterpret_cast<physx::PxVec3 const&>(*lin_pod);
        physx::PxVec3 const& ang = reinterpret_cast<physx::PxVec3 const&>(*ang_pod);
        float return_val = self_->projectVelocity(lin, ang);
        return return_val;
    }

    void PxSolverConstraintPrepDesc_delete(physx_PxSolverConstraintPrepDesc* self__pod) {
        physx::PxSolverConstraintPrepDesc* self_ = reinterpret_cast<physx::PxSolverConstraintPrepDesc*>(self__pod);
        delete self_;
    }

    uint8_t* PxConstraintAllocator_reserveConstraintData(physx_PxConstraintAllocator* self__pod, uint32_t byteSize) {
        physx::PxConstraintAllocator* self_ = reinterpret_cast<physx::PxConstraintAllocator*>(self__pod);
        uint8_t* return_val = self_->reserveConstraintData(byteSize);
        return return_val;
    }

    uint8_t* PxConstraintAllocator_reserveFrictionData(physx_PxConstraintAllocator* self__pod, uint32_t byteSize) {
        physx::PxConstraintAllocator* self_ = reinterpret_cast<physx::PxConstraintAllocator*>(self__pod);
        uint8_t* return_val = self_->reserveFrictionData(byteSize);
        return return_val;
    }

    void PxConstraintAllocator_delete(physx_PxConstraintAllocator* self__pod) {
        physx::PxConstraintAllocator* self_ = reinterpret_cast<physx::PxConstraintAllocator*>(self__pod);
        delete self_;
    }

    physx_PxArticulationLimit PxArticulationLimit_new() {
        PxArticulationLimit return_val;
        physx_PxArticulationLimit return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxArticulationLimit PxArticulationLimit_new_1(float low_, float high_) {
        PxArticulationLimit return_val(low_, high_);
        physx_PxArticulationLimit return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxArticulationDrive PxArticulationDrive_new() {
        PxArticulationDrive return_val;
        physx_PxArticulationDrive return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxArticulationDrive PxArticulationDrive_new_1(float stiffness_, float damping_, float maxForce_, PxArticulationDriveType driveType__pod) {
        auto driveType_ = static_cast<physx::PxArticulationDriveType::Enum>(driveType__pod);
        PxArticulationDrive return_val(stiffness_, damping_, maxForce_, driveType_);
        physx_PxArticulationDrive return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    float PxTGSSolverBodyVel_projectVelocity(physx_PxTGSSolverBodyVel const* self__pod, physx_PxVec3 const* lin_pod, physx_PxVec3 const* ang_pod) {
        physx::PxTGSSolverBodyVel const* self_ = reinterpret_cast<physx::PxTGSSolverBodyVel const*>(self__pod);
        physx::PxVec3 const& lin = reinterpret_cast<physx::PxVec3 const&>(*lin_pod);
        physx::PxVec3 const& ang = reinterpret_cast<physx::PxVec3 const&>(*ang_pod);
        float return_val = self_->projectVelocity(lin, ang);
        return return_val;
    }

    float PxTGSSolverBodyData_projectVelocity(physx_PxTGSSolverBodyData const* self__pod, physx_PxVec3 const* linear_pod, physx_PxVec3 const* angular_pod) {
        physx::PxTGSSolverBodyData const* self_ = reinterpret_cast<physx::PxTGSSolverBodyData const*>(self__pod);
        physx::PxVec3 const& linear = reinterpret_cast<physx::PxVec3 const&>(*linear_pod);
        physx::PxVec3 const& angular = reinterpret_cast<physx::PxVec3 const&>(*angular_pod);
        float return_val = self_->projectVelocity(linear, angular);
        return return_val;
    }

    void PxTGSSolverConstraintPrepDesc_delete(physx_PxTGSSolverConstraintPrepDesc* self__pod) {
        physx::PxTGSSolverConstraintPrepDesc* self_ = reinterpret_cast<physx::PxTGSSolverConstraintPrepDesc*>(self__pod);
        delete self_;
    }

    void PxArticulationAttachment_setRestLength(physx_PxArticulationAttachment* self__pod, float restLength) {
        physx::PxArticulationAttachment* self_ = reinterpret_cast<physx::PxArticulationAttachment*>(self__pod);
        self_->setRestLength(restLength);
    }

    float PxArticulationAttachment_getRestLength(physx_PxArticulationAttachment const* self__pod) {
        physx::PxArticulationAttachment const* self_ = reinterpret_cast<physx::PxArticulationAttachment const*>(self__pod);
        float return_val = self_->getRestLength();
        return return_val;
    }

    void PxArticulationAttachment_setLimitParameters(physx_PxArticulationAttachment* self__pod, physx_PxArticulationTendonLimit const* parameters_pod) {
        physx::PxArticulationAttachment* self_ = reinterpret_cast<physx::PxArticulationAttachment*>(self__pod);
        physx::PxArticulationTendonLimit const& parameters = reinterpret_cast<physx::PxArticulationTendonLimit const&>(*parameters_pod);
        self_->setLimitParameters(parameters);
    }

    physx_PxArticulationTendonLimit PxArticulationAttachment_getLimitParameters(physx_PxArticulationAttachment const* self__pod) {
        physx::PxArticulationAttachment const* self_ = reinterpret_cast<physx::PxArticulationAttachment const*>(self__pod);
        physx::PxArticulationTendonLimit return_val = self_->getLimitParameters();
        physx_PxArticulationTendonLimit return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxArticulationAttachment_setRelativeOffset(physx_PxArticulationAttachment* self__pod, physx_PxVec3 const* offset_pod) {
        physx::PxArticulationAttachment* self_ = reinterpret_cast<physx::PxArticulationAttachment*>(self__pod);
        physx::PxVec3 const& offset = reinterpret_cast<physx::PxVec3 const&>(*offset_pod);
        self_->setRelativeOffset(offset);
    }

    physx_PxVec3 PxArticulationAttachment_getRelativeOffset(physx_PxArticulationAttachment const* self__pod) {
        physx::PxArticulationAttachment const* self_ = reinterpret_cast<physx::PxArticulationAttachment const*>(self__pod);
        physx::PxVec3 return_val = self_->getRelativeOffset();
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxArticulationAttachment_setCoefficient(physx_PxArticulationAttachment* self__pod, float coefficient) {
        physx::PxArticulationAttachment* self_ = reinterpret_cast<physx::PxArticulationAttachment*>(self__pod);
        self_->setCoefficient(coefficient);
    }

    float PxArticulationAttachment_getCoefficient(physx_PxArticulationAttachment const* self__pod) {
        physx::PxArticulationAttachment const* self_ = reinterpret_cast<physx::PxArticulationAttachment const*>(self__pod);
        float return_val = self_->getCoefficient();
        return return_val;
    }

    physx_PxArticulationLink* PxArticulationAttachment_getLink(physx_PxArticulationAttachment const* self__pod) {
        physx::PxArticulationAttachment const* self_ = reinterpret_cast<physx::PxArticulationAttachment const*>(self__pod);
        physx::PxArticulationLink* return_val = self_->getLink();
        auto return_val_pod = reinterpret_cast<physx_PxArticulationLink*>(return_val);
        return return_val_pod;
    }

    physx_PxArticulationAttachment* PxArticulationAttachment_getParent(physx_PxArticulationAttachment const* self__pod) {
        physx::PxArticulationAttachment const* self_ = reinterpret_cast<physx::PxArticulationAttachment const*>(self__pod);
        physx::PxArticulationAttachment* return_val = self_->getParent();
        auto return_val_pod = reinterpret_cast<physx_PxArticulationAttachment*>(return_val);
        return return_val_pod;
    }

    bool PxArticulationAttachment_isLeaf(physx_PxArticulationAttachment const* self__pod) {
        physx::PxArticulationAttachment const* self_ = reinterpret_cast<physx::PxArticulationAttachment const*>(self__pod);
        bool return_val = self_->isLeaf();
        return return_val;
    }

    physx_PxArticulationSpatialTendon* PxArticulationAttachment_getTendon(physx_PxArticulationAttachment const* self__pod) {
        physx::PxArticulationAttachment const* self_ = reinterpret_cast<physx::PxArticulationAttachment const*>(self__pod);
        physx::PxArticulationSpatialTendon* return_val = self_->getTendon();
        auto return_val_pod = reinterpret_cast<physx_PxArticulationSpatialTendon*>(return_val);
        return return_val_pod;
    }

    void PxArticulationAttachment_release(physx_PxArticulationAttachment* self__pod) {
        physx::PxArticulationAttachment* self_ = reinterpret_cast<physx::PxArticulationAttachment*>(self__pod);
        self_->release();
    }

    char const* PxArticulationAttachment_getConcreteTypeName(physx_PxArticulationAttachment const* self__pod) {
        physx::PxArticulationAttachment const* self_ = reinterpret_cast<physx::PxArticulationAttachment const*>(self__pod);
        char const* return_val = self_->getConcreteTypeName();
        return return_val;
    }

    void PxArticulationTendonJoint_setCoefficient(physx_PxArticulationTendonJoint* self__pod, PxArticulationAxis axis_pod, float coefficient, float recipCoefficient) {
        physx::PxArticulationTendonJoint* self_ = reinterpret_cast<physx::PxArticulationTendonJoint*>(self__pod);
        auto axis = static_cast<physx::PxArticulationAxis::Enum>(axis_pod);
        self_->setCoefficient(axis, coefficient, recipCoefficient);
    }

    void PxArticulationTendonJoint_getCoefficient(physx_PxArticulationTendonJoint const* self__pod, PxArticulationAxis* axis_pod, float* coefficient_pod, float* recipCoefficient_pod) {
        physx::PxArticulationTendonJoint const* self_ = reinterpret_cast<physx::PxArticulationTendonJoint const*>(self__pod);
        physx::PxArticulationAxis::Enum& axis = reinterpret_cast<physx::PxArticulationAxis::Enum&>(*axis_pod);
        float& coefficient = *coefficient_pod;
        float& recipCoefficient = *recipCoefficient_pod;
        self_->getCoefficient(axis, coefficient, recipCoefficient);
    }

    physx_PxArticulationLink* PxArticulationTendonJoint_getLink(physx_PxArticulationTendonJoint const* self__pod) {
        physx::PxArticulationTendonJoint const* self_ = reinterpret_cast<physx::PxArticulationTendonJoint const*>(self__pod);
        physx::PxArticulationLink* return_val = self_->getLink();
        auto return_val_pod = reinterpret_cast<physx_PxArticulationLink*>(return_val);
        return return_val_pod;
    }

    physx_PxArticulationTendonJoint* PxArticulationTendonJoint_getParent(physx_PxArticulationTendonJoint const* self__pod) {
        physx::PxArticulationTendonJoint const* self_ = reinterpret_cast<physx::PxArticulationTendonJoint const*>(self__pod);
        physx::PxArticulationTendonJoint* return_val = self_->getParent();
        auto return_val_pod = reinterpret_cast<physx_PxArticulationTendonJoint*>(return_val);
        return return_val_pod;
    }

    physx_PxArticulationFixedTendon* PxArticulationTendonJoint_getTendon(physx_PxArticulationTendonJoint const* self__pod) {
        physx::PxArticulationTendonJoint const* self_ = reinterpret_cast<physx::PxArticulationTendonJoint const*>(self__pod);
        physx::PxArticulationFixedTendon* return_val = self_->getTendon();
        auto return_val_pod = reinterpret_cast<physx_PxArticulationFixedTendon*>(return_val);
        return return_val_pod;
    }

    void PxArticulationTendonJoint_release(physx_PxArticulationTendonJoint* self__pod) {
        physx::PxArticulationTendonJoint* self_ = reinterpret_cast<physx::PxArticulationTendonJoint*>(self__pod);
        self_->release();
    }

    char const* PxArticulationTendonJoint_getConcreteTypeName(physx_PxArticulationTendonJoint const* self__pod) {
        physx::PxArticulationTendonJoint const* self_ = reinterpret_cast<physx::PxArticulationTendonJoint const*>(self__pod);
        char const* return_val = self_->getConcreteTypeName();
        return return_val;
    }

    void PxArticulationTendon_setStiffness(physx_PxArticulationTendon* self__pod, float stiffness) {
        physx::PxArticulationTendon* self_ = reinterpret_cast<physx::PxArticulationTendon*>(self__pod);
        self_->setStiffness(stiffness);
    }

    float PxArticulationTendon_getStiffness(physx_PxArticulationTendon const* self__pod) {
        physx::PxArticulationTendon const* self_ = reinterpret_cast<physx::PxArticulationTendon const*>(self__pod);
        float return_val = self_->getStiffness();
        return return_val;
    }

    void PxArticulationTendon_setDamping(physx_PxArticulationTendon* self__pod, float damping) {
        physx::PxArticulationTendon* self_ = reinterpret_cast<physx::PxArticulationTendon*>(self__pod);
        self_->setDamping(damping);
    }

    float PxArticulationTendon_getDamping(physx_PxArticulationTendon const* self__pod) {
        physx::PxArticulationTendon const* self_ = reinterpret_cast<physx::PxArticulationTendon const*>(self__pod);
        float return_val = self_->getDamping();
        return return_val;
    }

    void PxArticulationTendon_setLimitStiffness(physx_PxArticulationTendon* self__pod, float stiffness) {
        physx::PxArticulationTendon* self_ = reinterpret_cast<physx::PxArticulationTendon*>(self__pod);
        self_->setLimitStiffness(stiffness);
    }

    float PxArticulationTendon_getLimitStiffness(physx_PxArticulationTendon const* self__pod) {
        physx::PxArticulationTendon const* self_ = reinterpret_cast<physx::PxArticulationTendon const*>(self__pod);
        float return_val = self_->getLimitStiffness();
        return return_val;
    }

    void PxArticulationTendon_setOffset(physx_PxArticulationTendon* self__pod, float offset, bool autowake) {
        physx::PxArticulationTendon* self_ = reinterpret_cast<physx::PxArticulationTendon*>(self__pod);
        self_->setOffset(offset, autowake);
    }

    float PxArticulationTendon_getOffset(physx_PxArticulationTendon const* self__pod) {
        physx::PxArticulationTendon const* self_ = reinterpret_cast<physx::PxArticulationTendon const*>(self__pod);
        float return_val = self_->getOffset();
        return return_val;
    }

    physx_PxArticulationReducedCoordinate* PxArticulationTendon_getArticulation(physx_PxArticulationTendon const* self__pod) {
        physx::PxArticulationTendon const* self_ = reinterpret_cast<physx::PxArticulationTendon const*>(self__pod);
        physx::PxArticulationReducedCoordinate* return_val = self_->getArticulation();
        auto return_val_pod = reinterpret_cast<physx_PxArticulationReducedCoordinate*>(return_val);
        return return_val_pod;
    }

    void PxArticulationTendon_release(physx_PxArticulationTendon* self__pod) {
        physx::PxArticulationTendon* self_ = reinterpret_cast<physx::PxArticulationTendon*>(self__pod);
        self_->release();
    }

    physx_PxArticulationAttachment* PxArticulationSpatialTendon_createAttachment(physx_PxArticulationSpatialTendon* self__pod, physx_PxArticulationAttachment* parent_pod, float coefficient, physx_PxVec3 relativeOffset_pod, physx_PxArticulationLink* link_pod) {
        physx::PxArticulationSpatialTendon* self_ = reinterpret_cast<physx::PxArticulationSpatialTendon*>(self__pod);
        physx::PxArticulationAttachment* parent = reinterpret_cast<physx::PxArticulationAttachment*>(parent_pod);
        physx::PxVec3 relativeOffset;
        memcpy(&relativeOffset, &relativeOffset_pod, sizeof(relativeOffset));
        physx::PxArticulationLink* link = reinterpret_cast<physx::PxArticulationLink*>(link_pod);
        physx::PxArticulationAttachment* return_val = self_->createAttachment(parent, coefficient, relativeOffset, link);
        auto return_val_pod = reinterpret_cast<physx_PxArticulationAttachment*>(return_val);
        return return_val_pod;
    }

    uint32_t PxArticulationSpatialTendon_getAttachments(physx_PxArticulationSpatialTendon const* self__pod, physx_PxArticulationAttachment** userBuffer_pod, uint32_t bufferSize, uint32_t startIndex) {
        physx::PxArticulationSpatialTendon const* self_ = reinterpret_cast<physx::PxArticulationSpatialTendon const*>(self__pod);
        physx::PxArticulationAttachment** userBuffer = reinterpret_cast<physx::PxArticulationAttachment**>(userBuffer_pod);
        uint32_t return_val = self_->getAttachments(userBuffer, bufferSize, startIndex);
        return return_val;
    }

    uint32_t PxArticulationSpatialTendon_getNbAttachments(physx_PxArticulationSpatialTendon const* self__pod) {
        physx::PxArticulationSpatialTendon const* self_ = reinterpret_cast<physx::PxArticulationSpatialTendon const*>(self__pod);
        uint32_t return_val = self_->getNbAttachments();
        return return_val;
    }

    char const* PxArticulationSpatialTendon_getConcreteTypeName(physx_PxArticulationSpatialTendon const* self__pod) {
        physx::PxArticulationSpatialTendon const* self_ = reinterpret_cast<physx::PxArticulationSpatialTendon const*>(self__pod);
        char const* return_val = self_->getConcreteTypeName();
        return return_val;
    }

    physx_PxArticulationTendonJoint* PxArticulationFixedTendon_createTendonJoint(physx_PxArticulationFixedTendon* self__pod, physx_PxArticulationTendonJoint* parent_pod, PxArticulationAxis axis_pod, float coefficient, float recipCoefficient, physx_PxArticulationLink* link_pod) {
        physx::PxArticulationFixedTendon* self_ = reinterpret_cast<physx::PxArticulationFixedTendon*>(self__pod);
        physx::PxArticulationTendonJoint* parent = reinterpret_cast<physx::PxArticulationTendonJoint*>(parent_pod);
        auto axis = static_cast<physx::PxArticulationAxis::Enum>(axis_pod);
        physx::PxArticulationLink* link = reinterpret_cast<physx::PxArticulationLink*>(link_pod);
        physx::PxArticulationTendonJoint* return_val = self_->createTendonJoint(parent, axis, coefficient, recipCoefficient, link);
        auto return_val_pod = reinterpret_cast<physx_PxArticulationTendonJoint*>(return_val);
        return return_val_pod;
    }

    uint32_t PxArticulationFixedTendon_getTendonJoints(physx_PxArticulationFixedTendon const* self__pod, physx_PxArticulationTendonJoint** userBuffer_pod, uint32_t bufferSize, uint32_t startIndex) {
        physx::PxArticulationFixedTendon const* self_ = reinterpret_cast<physx::PxArticulationFixedTendon const*>(self__pod);
        physx::PxArticulationTendonJoint** userBuffer = reinterpret_cast<physx::PxArticulationTendonJoint**>(userBuffer_pod);
        uint32_t return_val = self_->getTendonJoints(userBuffer, bufferSize, startIndex);
        return return_val;
    }

    uint32_t PxArticulationFixedTendon_getNbTendonJoints(physx_PxArticulationFixedTendon const* self__pod) {
        physx::PxArticulationFixedTendon const* self_ = reinterpret_cast<physx::PxArticulationFixedTendon const*>(self__pod);
        uint32_t return_val = self_->getNbTendonJoints();
        return return_val;
    }

    void PxArticulationFixedTendon_setRestLength(physx_PxArticulationFixedTendon* self__pod, float restLength) {
        physx::PxArticulationFixedTendon* self_ = reinterpret_cast<physx::PxArticulationFixedTendon*>(self__pod);
        self_->setRestLength(restLength);
    }

    float PxArticulationFixedTendon_getRestLength(physx_PxArticulationFixedTendon const* self__pod) {
        physx::PxArticulationFixedTendon const* self_ = reinterpret_cast<physx::PxArticulationFixedTendon const*>(self__pod);
        float return_val = self_->getRestLength();
        return return_val;
    }

    void PxArticulationFixedTendon_setLimitParameters(physx_PxArticulationFixedTendon* self__pod, physx_PxArticulationTendonLimit const* parameter_pod) {
        physx::PxArticulationFixedTendon* self_ = reinterpret_cast<physx::PxArticulationFixedTendon*>(self__pod);
        physx::PxArticulationTendonLimit const& parameter = reinterpret_cast<physx::PxArticulationTendonLimit const&>(*parameter_pod);
        self_->setLimitParameters(parameter);
    }

    physx_PxArticulationTendonLimit PxArticulationFixedTendon_getLimitParameters(physx_PxArticulationFixedTendon const* self__pod) {
        physx::PxArticulationFixedTendon const* self_ = reinterpret_cast<physx::PxArticulationFixedTendon const*>(self__pod);
        physx::PxArticulationTendonLimit return_val = self_->getLimitParameters();
        physx_PxArticulationTendonLimit return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    char const* PxArticulationFixedTendon_getConcreteTypeName(physx_PxArticulationFixedTendon const* self__pod) {
        physx::PxArticulationFixedTendon const* self_ = reinterpret_cast<physx::PxArticulationFixedTendon const*>(self__pod);
        char const* return_val = self_->getConcreteTypeName();
        return return_val;
    }

    physx_PxArticulationCache PxArticulationCache_new() {
        PxArticulationCache return_val;
        physx_PxArticulationCache return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxArticulationCache_release(physx_PxArticulationCache* self__pod) {
        physx::PxArticulationCache* self_ = reinterpret_cast<physx::PxArticulationCache*>(self__pod);
        self_->release();
    }

    void PxArticulationSensor_release(physx_PxArticulationSensor* self__pod) {
        physx::PxArticulationSensor* self_ = reinterpret_cast<physx::PxArticulationSensor*>(self__pod);
        self_->release();
    }

    physx_PxSpatialForce PxArticulationSensor_getForces(physx_PxArticulationSensor const* self__pod) {
        physx::PxArticulationSensor const* self_ = reinterpret_cast<physx::PxArticulationSensor const*>(self__pod);
        physx::PxSpatialForce return_val = self_->getForces();
        physx_PxSpatialForce return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxTransform PxArticulationSensor_getRelativePose(physx_PxArticulationSensor const* self__pod) {
        physx::PxArticulationSensor const* self_ = reinterpret_cast<physx::PxArticulationSensor const*>(self__pod);
        physx::PxTransform return_val = self_->getRelativePose();
        physx_PxTransform return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxArticulationSensor_setRelativePose(physx_PxArticulationSensor* self__pod, physx_PxTransform const* pose_pod) {
        physx::PxArticulationSensor* self_ = reinterpret_cast<physx::PxArticulationSensor*>(self__pod);
        physx::PxTransform const& pose = reinterpret_cast<physx::PxTransform const&>(*pose_pod);
        self_->setRelativePose(pose);
    }

    physx_PxArticulationLink* PxArticulationSensor_getLink(physx_PxArticulationSensor const* self__pod) {
        physx::PxArticulationSensor const* self_ = reinterpret_cast<physx::PxArticulationSensor const*>(self__pod);
        physx::PxArticulationLink* return_val = self_->getLink();
        auto return_val_pod = reinterpret_cast<physx_PxArticulationLink*>(return_val);
        return return_val_pod;
    }

    uint32_t PxArticulationSensor_getIndex(physx_PxArticulationSensor const* self__pod) {
        physx::PxArticulationSensor const* self_ = reinterpret_cast<physx::PxArticulationSensor const*>(self__pod);
        uint32_t return_val = self_->getIndex();
        return return_val;
    }

    physx_PxArticulationReducedCoordinate* PxArticulationSensor_getArticulation(physx_PxArticulationSensor const* self__pod) {
        physx::PxArticulationSensor const* self_ = reinterpret_cast<physx::PxArticulationSensor const*>(self__pod);
        physx::PxArticulationReducedCoordinate* return_val = self_->getArticulation();
        auto return_val_pod = reinterpret_cast<physx_PxArticulationReducedCoordinate*>(return_val);
        return return_val_pod;
    }

    PxArticulationSensorFlags PxArticulationSensor_getFlags(physx_PxArticulationSensor const* self__pod) {
        physx::PxArticulationSensor const* self_ = reinterpret_cast<physx::PxArticulationSensor const*>(self__pod);
        physx::PxArticulationSensorFlags return_val = self_->getFlags();
        PxArticulationSensorFlags return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxArticulationSensor_setFlag(physx_PxArticulationSensor* self__pod, PxArticulationSensorFlag flag_pod, bool enabled) {
        physx::PxArticulationSensor* self_ = reinterpret_cast<physx::PxArticulationSensor*>(self__pod);
        auto flag = static_cast<physx::PxArticulationSensorFlag::Enum>(flag_pod);
        self_->setFlag(flag, enabled);
    }

    char const* PxArticulationSensor_getConcreteTypeName(physx_PxArticulationSensor const* self__pod) {
        physx::PxArticulationSensor const* self_ = reinterpret_cast<physx::PxArticulationSensor const*>(self__pod);
        char const* return_val = self_->getConcreteTypeName();
        return return_val;
    }

    physx_PxScene* PxArticulationReducedCoordinate_getScene(physx_PxArticulationReducedCoordinate const* self__pod) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        physx::PxScene* return_val = self_->getScene();
        auto return_val_pod = reinterpret_cast<physx_PxScene*>(return_val);
        return return_val_pod;
    }

    void PxArticulationReducedCoordinate_setSolverIterationCounts(physx_PxArticulationReducedCoordinate* self__pod, uint32_t minPositionIters, uint32_t minVelocityIters) {
        physx::PxArticulationReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate*>(self__pod);
        self_->setSolverIterationCounts(minPositionIters, minVelocityIters);
    }

    void PxArticulationReducedCoordinate_getSolverIterationCounts(physx_PxArticulationReducedCoordinate const* self__pod, uint32_t* minPositionIters_pod, uint32_t* minVelocityIters_pod) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        uint32_t& minPositionIters = *minPositionIters_pod;
        uint32_t& minVelocityIters = *minVelocityIters_pod;
        self_->getSolverIterationCounts(minPositionIters, minVelocityIters);
    }

    bool PxArticulationReducedCoordinate_isSleeping(physx_PxArticulationReducedCoordinate const* self__pod) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        bool return_val = self_->isSleeping();
        return return_val;
    }

    void PxArticulationReducedCoordinate_setSleepThreshold(physx_PxArticulationReducedCoordinate* self__pod, float threshold) {
        physx::PxArticulationReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate*>(self__pod);
        self_->setSleepThreshold(threshold);
    }

    float PxArticulationReducedCoordinate_getSleepThreshold(physx_PxArticulationReducedCoordinate const* self__pod) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        float return_val = self_->getSleepThreshold();
        return return_val;
    }

    void PxArticulationReducedCoordinate_setStabilizationThreshold(physx_PxArticulationReducedCoordinate* self__pod, float threshold) {
        physx::PxArticulationReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate*>(self__pod);
        self_->setStabilizationThreshold(threshold);
    }

    float PxArticulationReducedCoordinate_getStabilizationThreshold(physx_PxArticulationReducedCoordinate const* self__pod) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        float return_val = self_->getStabilizationThreshold();
        return return_val;
    }

    void PxArticulationReducedCoordinate_setWakeCounter(physx_PxArticulationReducedCoordinate* self__pod, float wakeCounterValue) {
        physx::PxArticulationReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate*>(self__pod);
        self_->setWakeCounter(wakeCounterValue);
    }

    float PxArticulationReducedCoordinate_getWakeCounter(physx_PxArticulationReducedCoordinate const* self__pod) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        float return_val = self_->getWakeCounter();
        return return_val;
    }

    void PxArticulationReducedCoordinate_wakeUp(physx_PxArticulationReducedCoordinate* self__pod) {
        physx::PxArticulationReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate*>(self__pod);
        self_->wakeUp();
    }

    void PxArticulationReducedCoordinate_putToSleep(physx_PxArticulationReducedCoordinate* self__pod) {
        physx::PxArticulationReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate*>(self__pod);
        self_->putToSleep();
    }

    void PxArticulationReducedCoordinate_setMaxCOMLinearVelocity(physx_PxArticulationReducedCoordinate* self__pod, float maxLinearVelocity) {
        physx::PxArticulationReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate*>(self__pod);
        self_->setMaxCOMLinearVelocity(maxLinearVelocity);
    }

    float PxArticulationReducedCoordinate_getMaxCOMLinearVelocity(physx_PxArticulationReducedCoordinate const* self__pod) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        float return_val = self_->getMaxCOMLinearVelocity();
        return return_val;
    }

    void PxArticulationReducedCoordinate_setMaxCOMAngularVelocity(physx_PxArticulationReducedCoordinate* self__pod, float maxAngularVelocity) {
        physx::PxArticulationReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate*>(self__pod);
        self_->setMaxCOMAngularVelocity(maxAngularVelocity);
    }

    float PxArticulationReducedCoordinate_getMaxCOMAngularVelocity(physx_PxArticulationReducedCoordinate const* self__pod) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        float return_val = self_->getMaxCOMAngularVelocity();
        return return_val;
    }

    physx_PxArticulationLink* PxArticulationReducedCoordinate_createLink(physx_PxArticulationReducedCoordinate* self__pod, physx_PxArticulationLink* parent_pod, physx_PxTransform const* pose_pod) {
        physx::PxArticulationReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate*>(self__pod);
        physx::PxArticulationLink* parent = reinterpret_cast<physx::PxArticulationLink*>(parent_pod);
        physx::PxTransform const& pose = reinterpret_cast<physx::PxTransform const&>(*pose_pod);
        physx::PxArticulationLink* return_val = self_->createLink(parent, pose);
        auto return_val_pod = reinterpret_cast<physx_PxArticulationLink*>(return_val);
        return return_val_pod;
    }

    void PxArticulationReducedCoordinate_release(physx_PxArticulationReducedCoordinate* self__pod) {
        physx::PxArticulationReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate*>(self__pod);
        self_->release();
    }

    uint32_t PxArticulationReducedCoordinate_getNbLinks(physx_PxArticulationReducedCoordinate const* self__pod) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        uint32_t return_val = self_->getNbLinks();
        return return_val;
    }

    uint32_t PxArticulationReducedCoordinate_getLinks(physx_PxArticulationReducedCoordinate const* self__pod, physx_PxArticulationLink** userBuffer_pod, uint32_t bufferSize, uint32_t startIndex) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        physx::PxArticulationLink** userBuffer = reinterpret_cast<physx::PxArticulationLink**>(userBuffer_pod);
        uint32_t return_val = self_->getLinks(userBuffer, bufferSize, startIndex);
        return return_val;
    }

    uint32_t PxArticulationReducedCoordinate_getNbShapes(physx_PxArticulationReducedCoordinate const* self__pod) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        uint32_t return_val = self_->getNbShapes();
        return return_val;
    }

    void PxArticulationReducedCoordinate_setName(physx_PxArticulationReducedCoordinate* self__pod, char const* name) {
        physx::PxArticulationReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate*>(self__pod);
        self_->setName(name);
    }

    char const* PxArticulationReducedCoordinate_getName(physx_PxArticulationReducedCoordinate const* self__pod) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        char const* return_val = self_->getName();
        return return_val;
    }

    physx_PxBounds3 PxArticulationReducedCoordinate_getWorldBounds(physx_PxArticulationReducedCoordinate const* self__pod, float inflation) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        physx::PxBounds3 return_val = self_->getWorldBounds(inflation);
        physx_PxBounds3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxAggregate* PxArticulationReducedCoordinate_getAggregate(physx_PxArticulationReducedCoordinate const* self__pod) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        physx::PxAggregate* return_val = self_->getAggregate();
        auto return_val_pod = reinterpret_cast<physx_PxAggregate*>(return_val);
        return return_val_pod;
    }

    void PxArticulationReducedCoordinate_setArticulationFlags(physx_PxArticulationReducedCoordinate* self__pod, PxArticulationFlags flags_pod) {
        physx::PxArticulationReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate*>(self__pod);
        auto flags = physx::PxArticulationFlags(flags_pod);
        self_->setArticulationFlags(flags);
    }

    void PxArticulationReducedCoordinate_setArticulationFlag(physx_PxArticulationReducedCoordinate* self__pod, PxArticulationFlag flag_pod, bool value) {
        physx::PxArticulationReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate*>(self__pod);
        auto flag = static_cast<physx::PxArticulationFlag::Enum>(flag_pod);
        self_->setArticulationFlag(flag, value);
    }

    PxArticulationFlags PxArticulationReducedCoordinate_getArticulationFlags(physx_PxArticulationReducedCoordinate const* self__pod) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        physx::PxArticulationFlags return_val = self_->getArticulationFlags();
        PxArticulationFlags return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    uint32_t PxArticulationReducedCoordinate_getDofs(physx_PxArticulationReducedCoordinate const* self__pod) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        uint32_t return_val = self_->getDofs();
        return return_val;
    }

    physx_PxArticulationCache* PxArticulationReducedCoordinate_createCache(physx_PxArticulationReducedCoordinate const* self__pod) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        physx::PxArticulationCache* return_val = self_->createCache();
        auto return_val_pod = reinterpret_cast<physx_PxArticulationCache*>(return_val);
        return return_val_pod;
    }

    uint32_t PxArticulationReducedCoordinate_getCacheDataSize(physx_PxArticulationReducedCoordinate const* self__pod) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        uint32_t return_val = self_->getCacheDataSize();
        return return_val;
    }

    void PxArticulationReducedCoordinate_zeroCache(physx_PxArticulationReducedCoordinate const* self__pod, physx_PxArticulationCache* cache_pod) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        physx::PxArticulationCache& cache = reinterpret_cast<physx::PxArticulationCache&>(*cache_pod);
        self_->zeroCache(cache);
    }

    void PxArticulationReducedCoordinate_applyCache(physx_PxArticulationReducedCoordinate* self__pod, physx_PxArticulationCache* cache_pod, PxArticulationCacheFlags flags_pod, bool autowake) {
        physx::PxArticulationReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate*>(self__pod);
        physx::PxArticulationCache& cache = reinterpret_cast<physx::PxArticulationCache&>(*cache_pod);
        auto flags = physx::PxArticulationCacheFlags(flags_pod);
        self_->applyCache(cache, flags, autowake);
    }

    void PxArticulationReducedCoordinate_copyInternalStateToCache(physx_PxArticulationReducedCoordinate const* self__pod, physx_PxArticulationCache* cache_pod, PxArticulationCacheFlags flags_pod) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        physx::PxArticulationCache& cache = reinterpret_cast<physx::PxArticulationCache&>(*cache_pod);
        auto flags = physx::PxArticulationCacheFlags(flags_pod);
        self_->copyInternalStateToCache(cache, flags);
    }

    void PxArticulationReducedCoordinate_packJointData(physx_PxArticulationReducedCoordinate const* self__pod, float const* maximum, float* reduced) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        self_->packJointData(maximum, reduced);
    }

    void PxArticulationReducedCoordinate_unpackJointData(physx_PxArticulationReducedCoordinate const* self__pod, float const* reduced, float* maximum) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        self_->unpackJointData(reduced, maximum);
    }

    void PxArticulationReducedCoordinate_commonInit(physx_PxArticulationReducedCoordinate const* self__pod) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        self_->commonInit();
    }

    void PxArticulationReducedCoordinate_computeGeneralizedGravityForce(physx_PxArticulationReducedCoordinate const* self__pod, physx_PxArticulationCache* cache_pod) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        physx::PxArticulationCache& cache = reinterpret_cast<physx::PxArticulationCache&>(*cache_pod);
        self_->computeGeneralizedGravityForce(cache);
    }

    void PxArticulationReducedCoordinate_computeCoriolisAndCentrifugalForce(physx_PxArticulationReducedCoordinate const* self__pod, physx_PxArticulationCache* cache_pod) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        physx::PxArticulationCache& cache = reinterpret_cast<physx::PxArticulationCache&>(*cache_pod);
        self_->computeCoriolisAndCentrifugalForce(cache);
    }

    void PxArticulationReducedCoordinate_computeGeneralizedExternalForce(physx_PxArticulationReducedCoordinate const* self__pod, physx_PxArticulationCache* cache_pod) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        physx::PxArticulationCache& cache = reinterpret_cast<physx::PxArticulationCache&>(*cache_pod);
        self_->computeGeneralizedExternalForce(cache);
    }

    void PxArticulationReducedCoordinate_computeJointAcceleration(physx_PxArticulationReducedCoordinate const* self__pod, physx_PxArticulationCache* cache_pod) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        physx::PxArticulationCache& cache = reinterpret_cast<physx::PxArticulationCache&>(*cache_pod);
        self_->computeJointAcceleration(cache);
    }

    void PxArticulationReducedCoordinate_computeJointForce(physx_PxArticulationReducedCoordinate const* self__pod, physx_PxArticulationCache* cache_pod) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        physx::PxArticulationCache& cache = reinterpret_cast<physx::PxArticulationCache&>(*cache_pod);
        self_->computeJointForce(cache);
    }

    void PxArticulationReducedCoordinate_computeDenseJacobian(physx_PxArticulationReducedCoordinate const* self__pod, physx_PxArticulationCache* cache_pod, uint32_t* nRows_pod, uint32_t* nCols_pod) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        physx::PxArticulationCache& cache = reinterpret_cast<physx::PxArticulationCache&>(*cache_pod);
        uint32_t& nRows = *nRows_pod;
        uint32_t& nCols = *nCols_pod;
        self_->computeDenseJacobian(cache, nRows, nCols);
    }

    void PxArticulationReducedCoordinate_computeCoefficientMatrix(physx_PxArticulationReducedCoordinate const* self__pod, physx_PxArticulationCache* cache_pod) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        physx::PxArticulationCache& cache = reinterpret_cast<physx::PxArticulationCache&>(*cache_pod);
        self_->computeCoefficientMatrix(cache);
    }

    bool PxArticulationReducedCoordinate_computeLambda(physx_PxArticulationReducedCoordinate const* self__pod, physx_PxArticulationCache* cache_pod, physx_PxArticulationCache* initialState_pod, float const*const jointTorque, uint32_t maxIter) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        physx::PxArticulationCache& cache = reinterpret_cast<physx::PxArticulationCache&>(*cache_pod);
        physx::PxArticulationCache& initialState = reinterpret_cast<physx::PxArticulationCache&>(*initialState_pod);
        bool return_val = self_->computeLambda(cache, initialState, jointTorque, maxIter);
        return return_val;
    }

    void PxArticulationReducedCoordinate_computeGeneralizedMassMatrix(physx_PxArticulationReducedCoordinate const* self__pod, physx_PxArticulationCache* cache_pod) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        physx::PxArticulationCache& cache = reinterpret_cast<physx::PxArticulationCache&>(*cache_pod);
        self_->computeGeneralizedMassMatrix(cache);
    }

    void PxArticulationReducedCoordinate_addLoopJoint(physx_PxArticulationReducedCoordinate* self__pod, physx_PxConstraint* joint_pod) {
        physx::PxArticulationReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate*>(self__pod);
        physx::PxConstraint* joint = reinterpret_cast<physx::PxConstraint*>(joint_pod);
        self_->addLoopJoint(joint);
    }

    void PxArticulationReducedCoordinate_removeLoopJoint(physx_PxArticulationReducedCoordinate* self__pod, physx_PxConstraint* joint_pod) {
        physx::PxArticulationReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate*>(self__pod);
        physx::PxConstraint* joint = reinterpret_cast<physx::PxConstraint*>(joint_pod);
        self_->removeLoopJoint(joint);
    }

    uint32_t PxArticulationReducedCoordinate_getNbLoopJoints(physx_PxArticulationReducedCoordinate const* self__pod) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        uint32_t return_val = self_->getNbLoopJoints();
        return return_val;
    }

    uint32_t PxArticulationReducedCoordinate_getLoopJoints(physx_PxArticulationReducedCoordinate const* self__pod, physx_PxConstraint** userBuffer_pod, uint32_t bufferSize, uint32_t startIndex) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        physx::PxConstraint** userBuffer = reinterpret_cast<physx::PxConstraint**>(userBuffer_pod);
        uint32_t return_val = self_->getLoopJoints(userBuffer, bufferSize, startIndex);
        return return_val;
    }

    uint32_t PxArticulationReducedCoordinate_getCoefficientMatrixSize(physx_PxArticulationReducedCoordinate const* self__pod) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        uint32_t return_val = self_->getCoefficientMatrixSize();
        return return_val;
    }

    void PxArticulationReducedCoordinate_setRootGlobalPose(physx_PxArticulationReducedCoordinate* self__pod, physx_PxTransform const* pose_pod, bool autowake) {
        physx::PxArticulationReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate*>(self__pod);
        physx::PxTransform const& pose = reinterpret_cast<physx::PxTransform const&>(*pose_pod);
        self_->setRootGlobalPose(pose, autowake);
    }

    physx_PxTransform PxArticulationReducedCoordinate_getRootGlobalPose(physx_PxArticulationReducedCoordinate const* self__pod) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        physx::PxTransform return_val = self_->getRootGlobalPose();
        physx_PxTransform return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxArticulationReducedCoordinate_setRootLinearVelocity(physx_PxArticulationReducedCoordinate* self__pod, physx_PxVec3 const* linearVelocity_pod, bool autowake) {
        physx::PxArticulationReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate*>(self__pod);
        physx::PxVec3 const& linearVelocity = reinterpret_cast<physx::PxVec3 const&>(*linearVelocity_pod);
        self_->setRootLinearVelocity(linearVelocity, autowake);
    }

    physx_PxVec3 PxArticulationReducedCoordinate_getRootLinearVelocity(physx_PxArticulationReducedCoordinate const* self__pod) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        physx::PxVec3 return_val = self_->getRootLinearVelocity();
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxArticulationReducedCoordinate_setRootAngularVelocity(physx_PxArticulationReducedCoordinate* self__pod, physx_PxVec3 const* angularVelocity_pod, bool autowake) {
        physx::PxArticulationReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate*>(self__pod);
        physx::PxVec3 const& angularVelocity = reinterpret_cast<physx::PxVec3 const&>(*angularVelocity_pod);
        self_->setRootAngularVelocity(angularVelocity, autowake);
    }

    physx_PxVec3 PxArticulationReducedCoordinate_getRootAngularVelocity(physx_PxArticulationReducedCoordinate const* self__pod) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        physx::PxVec3 return_val = self_->getRootAngularVelocity();
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxSpatialVelocity PxArticulationReducedCoordinate_getLinkAcceleration(physx_PxArticulationReducedCoordinate* self__pod, uint32_t linkId) {
        physx::PxArticulationReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate*>(self__pod);
        physx::PxSpatialVelocity return_val = self_->getLinkAcceleration(linkId);
        physx_PxSpatialVelocity return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    uint32_t PxArticulationReducedCoordinate_getGpuArticulationIndex(physx_PxArticulationReducedCoordinate* self__pod) {
        physx::PxArticulationReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate*>(self__pod);
        uint32_t return_val = self_->getGpuArticulationIndex();
        return return_val;
    }

    physx_PxArticulationSpatialTendon* PxArticulationReducedCoordinate_createSpatialTendon(physx_PxArticulationReducedCoordinate* self__pod) {
        physx::PxArticulationReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate*>(self__pod);
        physx::PxArticulationSpatialTendon* return_val = self_->createSpatialTendon();
        auto return_val_pod = reinterpret_cast<physx_PxArticulationSpatialTendon*>(return_val);
        return return_val_pod;
    }

    physx_PxArticulationFixedTendon* PxArticulationReducedCoordinate_createFixedTendon(physx_PxArticulationReducedCoordinate* self__pod) {
        physx::PxArticulationReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate*>(self__pod);
        physx::PxArticulationFixedTendon* return_val = self_->createFixedTendon();
        auto return_val_pod = reinterpret_cast<physx_PxArticulationFixedTendon*>(return_val);
        return return_val_pod;
    }

    physx_PxArticulationSensor* PxArticulationReducedCoordinate_createSensor(physx_PxArticulationReducedCoordinate* self__pod, physx_PxArticulationLink* link_pod, physx_PxTransform const* relativePose_pod) {
        physx::PxArticulationReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate*>(self__pod);
        physx::PxArticulationLink* link = reinterpret_cast<physx::PxArticulationLink*>(link_pod);
        physx::PxTransform const& relativePose = reinterpret_cast<physx::PxTransform const&>(*relativePose_pod);
        physx::PxArticulationSensor* return_val = self_->createSensor(link, relativePose);
        auto return_val_pod = reinterpret_cast<physx_PxArticulationSensor*>(return_val);
        return return_val_pod;
    }

    uint32_t PxArticulationReducedCoordinate_getSpatialTendons(physx_PxArticulationReducedCoordinate const* self__pod, physx_PxArticulationSpatialTendon** userBuffer_pod, uint32_t bufferSize, uint32_t startIndex) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        physx::PxArticulationSpatialTendon** userBuffer = reinterpret_cast<physx::PxArticulationSpatialTendon**>(userBuffer_pod);
        uint32_t return_val = self_->getSpatialTendons(userBuffer, bufferSize, startIndex);
        return return_val;
    }

    uint32_t PxArticulationReducedCoordinate_getNbSpatialTendons(physx_PxArticulationReducedCoordinate* self__pod) {
        physx::PxArticulationReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate*>(self__pod);
        uint32_t return_val = self_->getNbSpatialTendons();
        return return_val;
    }

    uint32_t PxArticulationReducedCoordinate_getFixedTendons(physx_PxArticulationReducedCoordinate const* self__pod, physx_PxArticulationFixedTendon** userBuffer_pod, uint32_t bufferSize, uint32_t startIndex) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        physx::PxArticulationFixedTendon** userBuffer = reinterpret_cast<physx::PxArticulationFixedTendon**>(userBuffer_pod);
        uint32_t return_val = self_->getFixedTendons(userBuffer, bufferSize, startIndex);
        return return_val;
    }

    uint32_t PxArticulationReducedCoordinate_getNbFixedTendons(physx_PxArticulationReducedCoordinate* self__pod) {
        physx::PxArticulationReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate*>(self__pod);
        uint32_t return_val = self_->getNbFixedTendons();
        return return_val;
    }

    uint32_t PxArticulationReducedCoordinate_getSensors(physx_PxArticulationReducedCoordinate const* self__pod, physx_PxArticulationSensor** userBuffer_pod, uint32_t bufferSize, uint32_t startIndex) {
        physx::PxArticulationReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate const*>(self__pod);
        physx::PxArticulationSensor** userBuffer = reinterpret_cast<physx::PxArticulationSensor**>(userBuffer_pod);
        uint32_t return_val = self_->getSensors(userBuffer, bufferSize, startIndex);
        return return_val;
    }

    uint32_t PxArticulationReducedCoordinate_getNbSensors(physx_PxArticulationReducedCoordinate* self__pod) {
        physx::PxArticulationReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate*>(self__pod);
        uint32_t return_val = self_->getNbSensors();
        return return_val;
    }

    void PxArticulationReducedCoordinate_updateKinematic(physx_PxArticulationReducedCoordinate* self__pod, PxArticulationKinematicFlags flags_pod) {
        physx::PxArticulationReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationReducedCoordinate*>(self__pod);
        auto flags = physx::PxArticulationKinematicFlags(flags_pod);
        self_->updateKinematic(flags);
    }

    physx_PxArticulationLink* PxArticulationJointReducedCoordinate_getParentArticulationLink(physx_PxArticulationJointReducedCoordinate const* self__pod) {
        physx::PxArticulationJointReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationJointReducedCoordinate const*>(self__pod);
        physx::PxArticulationLink& return_val = self_->getParentArticulationLink();
        auto return_val_pod = reinterpret_cast<physx_PxArticulationLink*>(&return_val);
        return return_val_pod;
    }

    void PxArticulationJointReducedCoordinate_setParentPose(physx_PxArticulationJointReducedCoordinate* self__pod, physx_PxTransform const* pose_pod) {
        physx::PxArticulationJointReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationJointReducedCoordinate*>(self__pod);
        physx::PxTransform const& pose = reinterpret_cast<physx::PxTransform const&>(*pose_pod);
        self_->setParentPose(pose);
    }

    physx_PxTransform PxArticulationJointReducedCoordinate_getParentPose(physx_PxArticulationJointReducedCoordinate const* self__pod) {
        physx::PxArticulationJointReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationJointReducedCoordinate const*>(self__pod);
        physx::PxTransform return_val = self_->getParentPose();
        physx_PxTransform return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxArticulationLink* PxArticulationJointReducedCoordinate_getChildArticulationLink(physx_PxArticulationJointReducedCoordinate const* self__pod) {
        physx::PxArticulationJointReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationJointReducedCoordinate const*>(self__pod);
        physx::PxArticulationLink& return_val = self_->getChildArticulationLink();
        auto return_val_pod = reinterpret_cast<physx_PxArticulationLink*>(&return_val);
        return return_val_pod;
    }

    void PxArticulationJointReducedCoordinate_setChildPose(physx_PxArticulationJointReducedCoordinate* self__pod, physx_PxTransform const* pose_pod) {
        physx::PxArticulationJointReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationJointReducedCoordinate*>(self__pod);
        physx::PxTransform const& pose = reinterpret_cast<physx::PxTransform const&>(*pose_pod);
        self_->setChildPose(pose);
    }

    physx_PxTransform PxArticulationJointReducedCoordinate_getChildPose(physx_PxArticulationJointReducedCoordinate const* self__pod) {
        physx::PxArticulationJointReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationJointReducedCoordinate const*>(self__pod);
        physx::PxTransform return_val = self_->getChildPose();
        physx_PxTransform return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxArticulationJointReducedCoordinate_setJointType(physx_PxArticulationJointReducedCoordinate* self__pod, PxArticulationJointType jointType_pod) {
        physx::PxArticulationJointReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationJointReducedCoordinate*>(self__pod);
        auto jointType = static_cast<physx::PxArticulationJointType::Enum>(jointType_pod);
        self_->setJointType(jointType);
    }

    PxArticulationJointType PxArticulationJointReducedCoordinate_getJointType(physx_PxArticulationJointReducedCoordinate const* self__pod) {
        physx::PxArticulationJointReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationJointReducedCoordinate const*>(self__pod);
        physx::PxArticulationJointType::Enum return_val = self_->getJointType();
        PxArticulationJointType return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxArticulationJointReducedCoordinate_setMotion(physx_PxArticulationJointReducedCoordinate* self__pod, PxArticulationAxis axis_pod, PxArticulationMotion motion_pod) {
        physx::PxArticulationJointReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationJointReducedCoordinate*>(self__pod);
        auto axis = static_cast<physx::PxArticulationAxis::Enum>(axis_pod);
        auto motion = static_cast<physx::PxArticulationMotion::Enum>(motion_pod);
        self_->setMotion(axis, motion);
    }

    PxArticulationMotion PxArticulationJointReducedCoordinate_getMotion(physx_PxArticulationJointReducedCoordinate const* self__pod, PxArticulationAxis axis_pod) {
        physx::PxArticulationJointReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationJointReducedCoordinate const*>(self__pod);
        auto axis = static_cast<physx::PxArticulationAxis::Enum>(axis_pod);
        physx::PxArticulationMotion::Enum return_val = self_->getMotion(axis);
        PxArticulationMotion return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxArticulationJointReducedCoordinate_setLimitParams(physx_PxArticulationJointReducedCoordinate* self__pod, PxArticulationAxis axis_pod, physx_PxArticulationLimit const* limit_pod) {
        physx::PxArticulationJointReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationJointReducedCoordinate*>(self__pod);
        auto axis = static_cast<physx::PxArticulationAxis::Enum>(axis_pod);
        physx::PxArticulationLimit const& limit = reinterpret_cast<physx::PxArticulationLimit const&>(*limit_pod);
        self_->setLimitParams(axis, limit);
    }

    physx_PxArticulationLimit PxArticulationJointReducedCoordinate_getLimitParams(physx_PxArticulationJointReducedCoordinate const* self__pod, PxArticulationAxis axis_pod) {
        physx::PxArticulationJointReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationJointReducedCoordinate const*>(self__pod);
        auto axis = static_cast<physx::PxArticulationAxis::Enum>(axis_pod);
        physx::PxArticulationLimit return_val = self_->getLimitParams(axis);
        physx_PxArticulationLimit return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxArticulationJointReducedCoordinate_setDriveParams(physx_PxArticulationJointReducedCoordinate* self__pod, PxArticulationAxis axis_pod, physx_PxArticulationDrive const* drive_pod) {
        physx::PxArticulationJointReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationJointReducedCoordinate*>(self__pod);
        auto axis = static_cast<physx::PxArticulationAxis::Enum>(axis_pod);
        physx::PxArticulationDrive const& drive = reinterpret_cast<physx::PxArticulationDrive const&>(*drive_pod);
        self_->setDriveParams(axis, drive);
    }

    physx_PxArticulationDrive PxArticulationJointReducedCoordinate_getDriveParams(physx_PxArticulationJointReducedCoordinate const* self__pod, PxArticulationAxis axis_pod) {
        physx::PxArticulationJointReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationJointReducedCoordinate const*>(self__pod);
        auto axis = static_cast<physx::PxArticulationAxis::Enum>(axis_pod);
        physx::PxArticulationDrive return_val = self_->getDriveParams(axis);
        physx_PxArticulationDrive return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxArticulationJointReducedCoordinate_setDriveTarget(physx_PxArticulationJointReducedCoordinate* self__pod, PxArticulationAxis axis_pod, float target, bool autowake) {
        physx::PxArticulationJointReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationJointReducedCoordinate*>(self__pod);
        auto axis = static_cast<physx::PxArticulationAxis::Enum>(axis_pod);
        self_->setDriveTarget(axis, target, autowake);
    }

    float PxArticulationJointReducedCoordinate_getDriveTarget(physx_PxArticulationJointReducedCoordinate const* self__pod, PxArticulationAxis axis_pod) {
        physx::PxArticulationJointReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationJointReducedCoordinate const*>(self__pod);
        auto axis = static_cast<physx::PxArticulationAxis::Enum>(axis_pod);
        float return_val = self_->getDriveTarget(axis);
        return return_val;
    }

    void PxArticulationJointReducedCoordinate_setDriveVelocity(physx_PxArticulationJointReducedCoordinate* self__pod, PxArticulationAxis axis_pod, float targetVel, bool autowake) {
        physx::PxArticulationJointReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationJointReducedCoordinate*>(self__pod);
        auto axis = static_cast<physx::PxArticulationAxis::Enum>(axis_pod);
        self_->setDriveVelocity(axis, targetVel, autowake);
    }

    float PxArticulationJointReducedCoordinate_getDriveVelocity(physx_PxArticulationJointReducedCoordinate const* self__pod, PxArticulationAxis axis_pod) {
        physx::PxArticulationJointReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationJointReducedCoordinate const*>(self__pod);
        auto axis = static_cast<physx::PxArticulationAxis::Enum>(axis_pod);
        float return_val = self_->getDriveVelocity(axis);
        return return_val;
    }

    void PxArticulationJointReducedCoordinate_setArmature(physx_PxArticulationJointReducedCoordinate* self__pod, PxArticulationAxis axis_pod, float armature) {
        physx::PxArticulationJointReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationJointReducedCoordinate*>(self__pod);
        auto axis = static_cast<physx::PxArticulationAxis::Enum>(axis_pod);
        self_->setArmature(axis, armature);
    }

    float PxArticulationJointReducedCoordinate_getArmature(physx_PxArticulationJointReducedCoordinate const* self__pod, PxArticulationAxis axis_pod) {
        physx::PxArticulationJointReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationJointReducedCoordinate const*>(self__pod);
        auto axis = static_cast<physx::PxArticulationAxis::Enum>(axis_pod);
        float return_val = self_->getArmature(axis);
        return return_val;
    }

    void PxArticulationJointReducedCoordinate_setFrictionCoefficient(physx_PxArticulationJointReducedCoordinate* self__pod, float coefficient) {
        physx::PxArticulationJointReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationJointReducedCoordinate*>(self__pod);
        self_->setFrictionCoefficient(coefficient);
    }

    float PxArticulationJointReducedCoordinate_getFrictionCoefficient(physx_PxArticulationJointReducedCoordinate const* self__pod) {
        physx::PxArticulationJointReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationJointReducedCoordinate const*>(self__pod);
        float return_val = self_->getFrictionCoefficient();
        return return_val;
    }

    void PxArticulationJointReducedCoordinate_setMaxJointVelocity(physx_PxArticulationJointReducedCoordinate* self__pod, float maxJointV) {
        physx::PxArticulationJointReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationJointReducedCoordinate*>(self__pod);
        self_->setMaxJointVelocity(maxJointV);
    }

    float PxArticulationJointReducedCoordinate_getMaxJointVelocity(physx_PxArticulationJointReducedCoordinate const* self__pod) {
        physx::PxArticulationJointReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationJointReducedCoordinate const*>(self__pod);
        float return_val = self_->getMaxJointVelocity();
        return return_val;
    }

    void PxArticulationJointReducedCoordinate_setJointPosition(physx_PxArticulationJointReducedCoordinate* self__pod, PxArticulationAxis axis_pod, float jointPos) {
        physx::PxArticulationJointReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationJointReducedCoordinate*>(self__pod);
        auto axis = static_cast<physx::PxArticulationAxis::Enum>(axis_pod);
        self_->setJointPosition(axis, jointPos);
    }

    float PxArticulationJointReducedCoordinate_getJointPosition(physx_PxArticulationJointReducedCoordinate const* self__pod, PxArticulationAxis axis_pod) {
        physx::PxArticulationJointReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationJointReducedCoordinate const*>(self__pod);
        auto axis = static_cast<physx::PxArticulationAxis::Enum>(axis_pod);
        float return_val = self_->getJointPosition(axis);
        return return_val;
    }

    void PxArticulationJointReducedCoordinate_setJointVelocity(physx_PxArticulationJointReducedCoordinate* self__pod, PxArticulationAxis axis_pod, float jointVel) {
        physx::PxArticulationJointReducedCoordinate* self_ = reinterpret_cast<physx::PxArticulationJointReducedCoordinate*>(self__pod);
        auto axis = static_cast<physx::PxArticulationAxis::Enum>(axis_pod);
        self_->setJointVelocity(axis, jointVel);
    }

    float PxArticulationJointReducedCoordinate_getJointVelocity(physx_PxArticulationJointReducedCoordinate const* self__pod, PxArticulationAxis axis_pod) {
        physx::PxArticulationJointReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationJointReducedCoordinate const*>(self__pod);
        auto axis = static_cast<physx::PxArticulationAxis::Enum>(axis_pod);
        float return_val = self_->getJointVelocity(axis);
        return return_val;
    }

    char const* PxArticulationJointReducedCoordinate_getConcreteTypeName(physx_PxArticulationJointReducedCoordinate const* self__pod) {
        physx::PxArticulationJointReducedCoordinate const* self_ = reinterpret_cast<physx::PxArticulationJointReducedCoordinate const*>(self__pod);
        char const* return_val = self_->getConcreteTypeName();
        return return_val;
    }

    void PxShape_release(physx_PxShape* self__pod) {
        physx::PxShape* self_ = reinterpret_cast<physx::PxShape*>(self__pod);
        self_->release();
    }

    void PxShape_setGeometry(physx_PxShape* self__pod, physx_PxGeometry const* geometry_pod) {
        physx::PxShape* self_ = reinterpret_cast<physx::PxShape*>(self__pod);
        physx::PxGeometry const& geometry = reinterpret_cast<physx::PxGeometry const&>(*geometry_pod);
        self_->setGeometry(geometry);
    }

    physx_PxGeometry const* PxShape_getGeometry(physx_PxShape const* self__pod) {
        physx::PxShape const* self_ = reinterpret_cast<physx::PxShape const*>(self__pod);
        physx::PxGeometry const& return_val = self_->getGeometry();
        auto return_val_pod = reinterpret_cast<physx_PxGeometry const*>(&return_val);
        return return_val_pod;
    }

    physx_PxRigidActor* PxShape_getActor(physx_PxShape const* self__pod) {
        physx::PxShape const* self_ = reinterpret_cast<physx::PxShape const*>(self__pod);
        physx::PxRigidActor* return_val = self_->getActor();
        auto return_val_pod = reinterpret_cast<physx_PxRigidActor*>(return_val);
        return return_val_pod;
    }

    void PxShape_setLocalPose(physx_PxShape* self__pod, physx_PxTransform const* pose_pod) {
        physx::PxShape* self_ = reinterpret_cast<physx::PxShape*>(self__pod);
        physx::PxTransform const& pose = reinterpret_cast<physx::PxTransform const&>(*pose_pod);
        self_->setLocalPose(pose);
    }

    physx_PxTransform PxShape_getLocalPose(physx_PxShape const* self__pod) {
        physx::PxShape const* self_ = reinterpret_cast<physx::PxShape const*>(self__pod);
        physx::PxTransform return_val = self_->getLocalPose();
        physx_PxTransform return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxShape_setSimulationFilterData(physx_PxShape* self__pod, physx_PxFilterData const* data_pod) {
        physx::PxShape* self_ = reinterpret_cast<physx::PxShape*>(self__pod);
        physx::PxFilterData const& data = reinterpret_cast<physx::PxFilterData const&>(*data_pod);
        self_->setSimulationFilterData(data);
    }

    physx_PxFilterData PxShape_getSimulationFilterData(physx_PxShape const* self__pod) {
        physx::PxShape const* self_ = reinterpret_cast<physx::PxShape const*>(self__pod);
        physx::PxFilterData return_val = self_->getSimulationFilterData();
        physx_PxFilterData return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxShape_setQueryFilterData(physx_PxShape* self__pod, physx_PxFilterData const* data_pod) {
        physx::PxShape* self_ = reinterpret_cast<physx::PxShape*>(self__pod);
        physx::PxFilterData const& data = reinterpret_cast<physx::PxFilterData const&>(*data_pod);
        self_->setQueryFilterData(data);
    }

    physx_PxFilterData PxShape_getQueryFilterData(physx_PxShape const* self__pod) {
        physx::PxShape const* self_ = reinterpret_cast<physx::PxShape const*>(self__pod);
        physx::PxFilterData return_val = self_->getQueryFilterData();
        physx_PxFilterData return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxShape_setMaterials(physx_PxShape* self__pod, physx_PxMaterial* const* materials_pod, uint16_t materialCount) {
        physx::PxShape* self_ = reinterpret_cast<physx::PxShape*>(self__pod);
        physx::PxMaterial* const* materials = reinterpret_cast<physx::PxMaterial* const*>(materials_pod);
        self_->setMaterials(materials, materialCount);
    }

    uint16_t PxShape_getNbMaterials(physx_PxShape const* self__pod) {
        physx::PxShape const* self_ = reinterpret_cast<physx::PxShape const*>(self__pod);
        uint16_t return_val = self_->getNbMaterials();
        return return_val;
    }

    uint32_t PxShape_getMaterials(physx_PxShape const* self__pod, physx_PxMaterial** userBuffer_pod, uint32_t bufferSize, uint32_t startIndex) {
        physx::PxShape const* self_ = reinterpret_cast<physx::PxShape const*>(self__pod);
        physx::PxMaterial** userBuffer = reinterpret_cast<physx::PxMaterial**>(userBuffer_pod);
        uint32_t return_val = self_->getMaterials(userBuffer, bufferSize, startIndex);
        return return_val;
    }

    physx_PxBaseMaterial* PxShape_getMaterialFromInternalFaceIndex(physx_PxShape const* self__pod, uint32_t faceIndex) {
        physx::PxShape const* self_ = reinterpret_cast<physx::PxShape const*>(self__pod);
        physx::PxBaseMaterial* return_val = self_->getMaterialFromInternalFaceIndex(faceIndex);
        auto return_val_pod = reinterpret_cast<physx_PxBaseMaterial*>(return_val);
        return return_val_pod;
    }

    void PxShape_setContactOffset(physx_PxShape* self__pod, float contactOffset) {
        physx::PxShape* self_ = reinterpret_cast<physx::PxShape*>(self__pod);
        self_->setContactOffset(contactOffset);
    }

    float PxShape_getContactOffset(physx_PxShape const* self__pod) {
        physx::PxShape const* self_ = reinterpret_cast<physx::PxShape const*>(self__pod);
        float return_val = self_->getContactOffset();
        return return_val;
    }

    void PxShape_setRestOffset(physx_PxShape* self__pod, float restOffset) {
        physx::PxShape* self_ = reinterpret_cast<physx::PxShape*>(self__pod);
        self_->setRestOffset(restOffset);
    }

    float PxShape_getRestOffset(physx_PxShape const* self__pod) {
        physx::PxShape const* self_ = reinterpret_cast<physx::PxShape const*>(self__pod);
        float return_val = self_->getRestOffset();
        return return_val;
    }

    void PxShape_setDensityForFluid(physx_PxShape* self__pod, float densityForFluid) {
        physx::PxShape* self_ = reinterpret_cast<physx::PxShape*>(self__pod);
        self_->setDensityForFluid(densityForFluid);
    }

    float PxShape_getDensityForFluid(physx_PxShape const* self__pod) {
        physx::PxShape const* self_ = reinterpret_cast<physx::PxShape const*>(self__pod);
        float return_val = self_->getDensityForFluid();
        return return_val;
    }

    void PxShape_setTorsionalPatchRadius(physx_PxShape* self__pod, float radius) {
        physx::PxShape* self_ = reinterpret_cast<physx::PxShape*>(self__pod);
        self_->setTorsionalPatchRadius(radius);
    }

    float PxShape_getTorsionalPatchRadius(physx_PxShape const* self__pod) {
        physx::PxShape const* self_ = reinterpret_cast<physx::PxShape const*>(self__pod);
        float return_val = self_->getTorsionalPatchRadius();
        return return_val;
    }

    void PxShape_setMinTorsionalPatchRadius(physx_PxShape* self__pod, float radius) {
        physx::PxShape* self_ = reinterpret_cast<physx::PxShape*>(self__pod);
        self_->setMinTorsionalPatchRadius(radius);
    }

    float PxShape_getMinTorsionalPatchRadius(physx_PxShape const* self__pod) {
        physx::PxShape const* self_ = reinterpret_cast<physx::PxShape const*>(self__pod);
        float return_val = self_->getMinTorsionalPatchRadius();
        return return_val;
    }

    void PxShape_setFlag(physx_PxShape* self__pod, PxShapeFlag flag_pod, bool value) {
        physx::PxShape* self_ = reinterpret_cast<physx::PxShape*>(self__pod);
        auto flag = static_cast<physx::PxShapeFlag::Enum>(flag_pod);
        self_->setFlag(flag, value);
    }

    void PxShape_setFlags(physx_PxShape* self__pod, PxShapeFlags inFlags_pod) {
        physx::PxShape* self_ = reinterpret_cast<physx::PxShape*>(self__pod);
        auto inFlags = physx::PxShapeFlags(inFlags_pod);
        self_->setFlags(inFlags);
    }

    PxShapeFlags PxShape_getFlags(physx_PxShape const* self__pod) {
        physx::PxShape const* self_ = reinterpret_cast<physx::PxShape const*>(self__pod);
        physx::PxShapeFlags return_val = self_->getFlags();
        PxShapeFlags return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxShape_isExclusive(physx_PxShape const* self__pod) {
        physx::PxShape const* self_ = reinterpret_cast<physx::PxShape const*>(self__pod);
        bool return_val = self_->isExclusive();
        return return_val;
    }

    void PxShape_setName(physx_PxShape* self__pod, char const* name) {
        physx::PxShape* self_ = reinterpret_cast<physx::PxShape*>(self__pod);
        self_->setName(name);
    }

    char const* PxShape_getName(physx_PxShape const* self__pod) {
        physx::PxShape const* self_ = reinterpret_cast<physx::PxShape const*>(self__pod);
        char const* return_val = self_->getName();
        return return_val;
    }

    char const* PxShape_getConcreteTypeName(physx_PxShape const* self__pod) {
        physx::PxShape const* self_ = reinterpret_cast<physx::PxShape const*>(self__pod);
        char const* return_val = self_->getConcreteTypeName();
        return return_val;
    }

    void PxRigidActor_release(physx_PxRigidActor* self__pod) {
        physx::PxRigidActor* self_ = reinterpret_cast<physx::PxRigidActor*>(self__pod);
        self_->release();
    }

    uint32_t PxRigidActor_getInternalActorIndex(physx_PxRigidActor const* self__pod) {
        physx::PxRigidActor const* self_ = reinterpret_cast<physx::PxRigidActor const*>(self__pod);
        uint32_t return_val = self_->getInternalActorIndex();
        return return_val;
    }

    physx_PxTransform PxRigidActor_getGlobalPose(physx_PxRigidActor const* self__pod) {
        physx::PxRigidActor const* self_ = reinterpret_cast<physx::PxRigidActor const*>(self__pod);
        physx::PxTransform return_val = self_->getGlobalPose();
        physx_PxTransform return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxRigidActor_setGlobalPose(physx_PxRigidActor* self__pod, physx_PxTransform const* pose_pod, bool autowake) {
        physx::PxRigidActor* self_ = reinterpret_cast<physx::PxRigidActor*>(self__pod);
        physx::PxTransform const& pose = reinterpret_cast<physx::PxTransform const&>(*pose_pod);
        self_->setGlobalPose(pose, autowake);
    }

    bool PxRigidActor_attachShape(physx_PxRigidActor* self__pod, physx_PxShape* shape_pod) {
        physx::PxRigidActor* self_ = reinterpret_cast<physx::PxRigidActor*>(self__pod);
        physx::PxShape& shape = reinterpret_cast<physx::PxShape&>(*shape_pod);
        bool return_val = self_->attachShape(shape);
        return return_val;
    }

    void PxRigidActor_detachShape(physx_PxRigidActor* self__pod, physx_PxShape* shape_pod, bool wakeOnLostTouch) {
        physx::PxRigidActor* self_ = reinterpret_cast<physx::PxRigidActor*>(self__pod);
        physx::PxShape& shape = reinterpret_cast<physx::PxShape&>(*shape_pod);
        self_->detachShape(shape, wakeOnLostTouch);
    }

    uint32_t PxRigidActor_getNbShapes(physx_PxRigidActor const* self__pod) {
        physx::PxRigidActor const* self_ = reinterpret_cast<physx::PxRigidActor const*>(self__pod);
        uint32_t return_val = self_->getNbShapes();
        return return_val;
    }

    uint32_t PxRigidActor_getShapes(physx_PxRigidActor const* self__pod, physx_PxShape** userBuffer_pod, uint32_t bufferSize, uint32_t startIndex) {
        physx::PxRigidActor const* self_ = reinterpret_cast<physx::PxRigidActor const*>(self__pod);
        physx::PxShape** userBuffer = reinterpret_cast<physx::PxShape**>(userBuffer_pod);
        uint32_t return_val = self_->getShapes(userBuffer, bufferSize, startIndex);
        return return_val;
    }

    uint32_t PxRigidActor_getNbConstraints(physx_PxRigidActor const* self__pod) {
        physx::PxRigidActor const* self_ = reinterpret_cast<physx::PxRigidActor const*>(self__pod);
        uint32_t return_val = self_->getNbConstraints();
        return return_val;
    }

    uint32_t PxRigidActor_getConstraints(physx_PxRigidActor const* self__pod, physx_PxConstraint** userBuffer_pod, uint32_t bufferSize, uint32_t startIndex) {
        physx::PxRigidActor const* self_ = reinterpret_cast<physx::PxRigidActor const*>(self__pod);
        physx::PxConstraint** userBuffer = reinterpret_cast<physx::PxConstraint**>(userBuffer_pod);
        uint32_t return_val = self_->getConstraints(userBuffer, bufferSize, startIndex);
        return return_val;
    }

    physx_PxNodeIndex PxNodeIndex_new(uint32_t id, uint32_t articLinkId) {
        PxNodeIndex return_val(id, articLinkId);
        physx_PxNodeIndex return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxNodeIndex PxNodeIndex_new_1(uint32_t id) {
        PxNodeIndex return_val(id);
        physx_PxNodeIndex return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    uint32_t PxNodeIndex_index(physx_PxNodeIndex const* self__pod) {
        physx::PxNodeIndex const* self_ = reinterpret_cast<physx::PxNodeIndex const*>(self__pod);
        uint32_t return_val = self_->index();
        return return_val;
    }

    uint32_t PxNodeIndex_articulationLinkId(physx_PxNodeIndex const* self__pod) {
        physx::PxNodeIndex const* self_ = reinterpret_cast<physx::PxNodeIndex const*>(self__pod);
        uint32_t return_val = self_->articulationLinkId();
        return return_val;
    }

    uint32_t PxNodeIndex_isArticulation(physx_PxNodeIndex const* self__pod) {
        physx::PxNodeIndex const* self_ = reinterpret_cast<physx::PxNodeIndex const*>(self__pod);
        uint32_t return_val = self_->isArticulation();
        return return_val;
    }

    bool PxNodeIndex_isStaticBody(physx_PxNodeIndex const* self__pod) {
        physx::PxNodeIndex const* self_ = reinterpret_cast<physx::PxNodeIndex const*>(self__pod);
        bool return_val = self_->isStaticBody();
        return return_val;
    }

    bool PxNodeIndex_isValid(physx_PxNodeIndex const* self__pod) {
        physx::PxNodeIndex const* self_ = reinterpret_cast<physx::PxNodeIndex const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    void PxNodeIndex_setIndices(physx_PxNodeIndex* self__pod, uint32_t index, uint32_t articLinkId) {
        physx::PxNodeIndex* self_ = reinterpret_cast<physx::PxNodeIndex*>(self__pod);
        self_->setIndices(index, articLinkId);
    }

    void PxNodeIndex_setIndices_1(physx_PxNodeIndex* self__pod, uint32_t index) {
        physx::PxNodeIndex* self_ = reinterpret_cast<physx::PxNodeIndex*>(self__pod);
        self_->setIndices(index);
    }

    uint64_t PxNodeIndex_getInd(physx_PxNodeIndex const* self__pod) {
        physx::PxNodeIndex const* self_ = reinterpret_cast<physx::PxNodeIndex const*>(self__pod);
        uint64_t return_val = self_->getInd();
        return return_val;
    }

    void PxRigidBody_setCMassLocalPose(physx_PxRigidBody* self__pod, physx_PxTransform const* pose_pod) {
        physx::PxRigidBody* self_ = reinterpret_cast<physx::PxRigidBody*>(self__pod);
        physx::PxTransform const& pose = reinterpret_cast<physx::PxTransform const&>(*pose_pod);
        self_->setCMassLocalPose(pose);
    }

    physx_PxTransform PxRigidBody_getCMassLocalPose(physx_PxRigidBody const* self__pod) {
        physx::PxRigidBody const* self_ = reinterpret_cast<physx::PxRigidBody const*>(self__pod);
        physx::PxTransform return_val = self_->getCMassLocalPose();
        physx_PxTransform return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxRigidBody_setMass(physx_PxRigidBody* self__pod, float mass) {
        physx::PxRigidBody* self_ = reinterpret_cast<physx::PxRigidBody*>(self__pod);
        self_->setMass(mass);
    }

    float PxRigidBody_getMass(physx_PxRigidBody const* self__pod) {
        physx::PxRigidBody const* self_ = reinterpret_cast<physx::PxRigidBody const*>(self__pod);
        float return_val = self_->getMass();
        return return_val;
    }

    float PxRigidBody_getInvMass(physx_PxRigidBody const* self__pod) {
        physx::PxRigidBody const* self_ = reinterpret_cast<physx::PxRigidBody const*>(self__pod);
        float return_val = self_->getInvMass();
        return return_val;
    }

    void PxRigidBody_setMassSpaceInertiaTensor(physx_PxRigidBody* self__pod, physx_PxVec3 const* m_pod) {
        physx::PxRigidBody* self_ = reinterpret_cast<physx::PxRigidBody*>(self__pod);
        physx::PxVec3 const& m = reinterpret_cast<physx::PxVec3 const&>(*m_pod);
        self_->setMassSpaceInertiaTensor(m);
    }

    physx_PxVec3 PxRigidBody_getMassSpaceInertiaTensor(physx_PxRigidBody const* self__pod) {
        physx::PxRigidBody const* self_ = reinterpret_cast<physx::PxRigidBody const*>(self__pod);
        physx::PxVec3 return_val = self_->getMassSpaceInertiaTensor();
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3 PxRigidBody_getMassSpaceInvInertiaTensor(physx_PxRigidBody const* self__pod) {
        physx::PxRigidBody const* self_ = reinterpret_cast<physx::PxRigidBody const*>(self__pod);
        physx::PxVec3 return_val = self_->getMassSpaceInvInertiaTensor();
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxRigidBody_setLinearDamping(physx_PxRigidBody* self__pod, float linDamp) {
        physx::PxRigidBody* self_ = reinterpret_cast<physx::PxRigidBody*>(self__pod);
        self_->setLinearDamping(linDamp);
    }

    float PxRigidBody_getLinearDamping(physx_PxRigidBody const* self__pod) {
        physx::PxRigidBody const* self_ = reinterpret_cast<physx::PxRigidBody const*>(self__pod);
        float return_val = self_->getLinearDamping();
        return return_val;
    }

    void PxRigidBody_setAngularDamping(physx_PxRigidBody* self__pod, float angDamp) {
        physx::PxRigidBody* self_ = reinterpret_cast<physx::PxRigidBody*>(self__pod);
        self_->setAngularDamping(angDamp);
    }

    float PxRigidBody_getAngularDamping(physx_PxRigidBody const* self__pod) {
        physx::PxRigidBody const* self_ = reinterpret_cast<physx::PxRigidBody const*>(self__pod);
        float return_val = self_->getAngularDamping();
        return return_val;
    }

    physx_PxVec3 PxRigidBody_getLinearVelocity(physx_PxRigidBody const* self__pod) {
        physx::PxRigidBody const* self_ = reinterpret_cast<physx::PxRigidBody const*>(self__pod);
        physx::PxVec3 return_val = self_->getLinearVelocity();
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3 PxRigidBody_getAngularVelocity(physx_PxRigidBody const* self__pod) {
        physx::PxRigidBody const* self_ = reinterpret_cast<physx::PxRigidBody const*>(self__pod);
        physx::PxVec3 return_val = self_->getAngularVelocity();
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxRigidBody_setMaxLinearVelocity(physx_PxRigidBody* self__pod, float maxLinVel) {
        physx::PxRigidBody* self_ = reinterpret_cast<physx::PxRigidBody*>(self__pod);
        self_->setMaxLinearVelocity(maxLinVel);
    }

    float PxRigidBody_getMaxLinearVelocity(physx_PxRigidBody const* self__pod) {
        physx::PxRigidBody const* self_ = reinterpret_cast<physx::PxRigidBody const*>(self__pod);
        float return_val = self_->getMaxLinearVelocity();
        return return_val;
    }

    void PxRigidBody_setMaxAngularVelocity(physx_PxRigidBody* self__pod, float maxAngVel) {
        physx::PxRigidBody* self_ = reinterpret_cast<physx::PxRigidBody*>(self__pod);
        self_->setMaxAngularVelocity(maxAngVel);
    }

    float PxRigidBody_getMaxAngularVelocity(physx_PxRigidBody const* self__pod) {
        physx::PxRigidBody const* self_ = reinterpret_cast<physx::PxRigidBody const*>(self__pod);
        float return_val = self_->getMaxAngularVelocity();
        return return_val;
    }

    void PxRigidBody_addForce(physx_PxRigidBody* self__pod, physx_PxVec3 const* force_pod, PxForceMode mode_pod, bool autowake) {
        physx::PxRigidBody* self_ = reinterpret_cast<physx::PxRigidBody*>(self__pod);
        physx::PxVec3 const& force = reinterpret_cast<physx::PxVec3 const&>(*force_pod);
        auto mode = static_cast<physx::PxForceMode::Enum>(mode_pod);
        self_->addForce(force, mode, autowake);
    }

    void PxRigidBody_addTorque(physx_PxRigidBody* self__pod, physx_PxVec3 const* torque_pod, PxForceMode mode_pod, bool autowake) {
        physx::PxRigidBody* self_ = reinterpret_cast<physx::PxRigidBody*>(self__pod);
        physx::PxVec3 const& torque = reinterpret_cast<physx::PxVec3 const&>(*torque_pod);
        auto mode = static_cast<physx::PxForceMode::Enum>(mode_pod);
        self_->addTorque(torque, mode, autowake);
    }

    void PxRigidBody_clearForce(physx_PxRigidBody* self__pod, PxForceMode mode_pod) {
        physx::PxRigidBody* self_ = reinterpret_cast<physx::PxRigidBody*>(self__pod);
        auto mode = static_cast<physx::PxForceMode::Enum>(mode_pod);
        self_->clearForce(mode);
    }

    void PxRigidBody_clearTorque(physx_PxRigidBody* self__pod, PxForceMode mode_pod) {
        physx::PxRigidBody* self_ = reinterpret_cast<physx::PxRigidBody*>(self__pod);
        auto mode = static_cast<physx::PxForceMode::Enum>(mode_pod);
        self_->clearTorque(mode);
    }

    void PxRigidBody_setForceAndTorque(physx_PxRigidBody* self__pod, physx_PxVec3 const* force_pod, physx_PxVec3 const* torque_pod, PxForceMode mode_pod) {
        physx::PxRigidBody* self_ = reinterpret_cast<physx::PxRigidBody*>(self__pod);
        physx::PxVec3 const& force = reinterpret_cast<physx::PxVec3 const&>(*force_pod);
        physx::PxVec3 const& torque = reinterpret_cast<physx::PxVec3 const&>(*torque_pod);
        auto mode = static_cast<physx::PxForceMode::Enum>(mode_pod);
        self_->setForceAndTorque(force, torque, mode);
    }

    void PxRigidBody_setRigidBodyFlag(physx_PxRigidBody* self__pod, PxRigidBodyFlag flag_pod, bool value) {
        physx::PxRigidBody* self_ = reinterpret_cast<physx::PxRigidBody*>(self__pod);
        auto flag = static_cast<physx::PxRigidBodyFlag::Enum>(flag_pod);
        self_->setRigidBodyFlag(flag, value);
    }

    void PxRigidBody_setRigidBodyFlags(physx_PxRigidBody* self__pod, PxRigidBodyFlags inFlags_pod) {
        physx::PxRigidBody* self_ = reinterpret_cast<physx::PxRigidBody*>(self__pod);
        auto inFlags = physx::PxRigidBodyFlags(inFlags_pod);
        self_->setRigidBodyFlags(inFlags);
    }

    PxRigidBodyFlags PxRigidBody_getRigidBodyFlags(physx_PxRigidBody const* self__pod) {
        physx::PxRigidBody const* self_ = reinterpret_cast<physx::PxRigidBody const*>(self__pod);
        physx::PxRigidBodyFlags return_val = self_->getRigidBodyFlags();
        PxRigidBodyFlags return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxRigidBody_setMinCCDAdvanceCoefficient(physx_PxRigidBody* self__pod, float advanceCoefficient) {
        physx::PxRigidBody* self_ = reinterpret_cast<physx::PxRigidBody*>(self__pod);
        self_->setMinCCDAdvanceCoefficient(advanceCoefficient);
    }

    float PxRigidBody_getMinCCDAdvanceCoefficient(physx_PxRigidBody const* self__pod) {
        physx::PxRigidBody const* self_ = reinterpret_cast<physx::PxRigidBody const*>(self__pod);
        float return_val = self_->getMinCCDAdvanceCoefficient();
        return return_val;
    }

    void PxRigidBody_setMaxDepenetrationVelocity(physx_PxRigidBody* self__pod, float biasClamp) {
        physx::PxRigidBody* self_ = reinterpret_cast<physx::PxRigidBody*>(self__pod);
        self_->setMaxDepenetrationVelocity(biasClamp);
    }

    float PxRigidBody_getMaxDepenetrationVelocity(physx_PxRigidBody const* self__pod) {
        physx::PxRigidBody const* self_ = reinterpret_cast<physx::PxRigidBody const*>(self__pod);
        float return_val = self_->getMaxDepenetrationVelocity();
        return return_val;
    }

    void PxRigidBody_setMaxContactImpulse(physx_PxRigidBody* self__pod, float maxImpulse) {
        physx::PxRigidBody* self_ = reinterpret_cast<physx::PxRigidBody*>(self__pod);
        self_->setMaxContactImpulse(maxImpulse);
    }

    float PxRigidBody_getMaxContactImpulse(physx_PxRigidBody const* self__pod) {
        physx::PxRigidBody const* self_ = reinterpret_cast<physx::PxRigidBody const*>(self__pod);
        float return_val = self_->getMaxContactImpulse();
        return return_val;
    }

    void PxRigidBody_setContactSlopCoefficient(physx_PxRigidBody* self__pod, float slopCoefficient) {
        physx::PxRigidBody* self_ = reinterpret_cast<physx::PxRigidBody*>(self__pod);
        self_->setContactSlopCoefficient(slopCoefficient);
    }

    float PxRigidBody_getContactSlopCoefficient(physx_PxRigidBody const* self__pod) {
        physx::PxRigidBody const* self_ = reinterpret_cast<physx::PxRigidBody const*>(self__pod);
        float return_val = self_->getContactSlopCoefficient();
        return return_val;
    }

    physx_PxNodeIndex PxRigidBody_getInternalIslandNodeIndex(physx_PxRigidBody const* self__pod) {
        physx::PxRigidBody const* self_ = reinterpret_cast<physx::PxRigidBody const*>(self__pod);
        physx::PxNodeIndex return_val = self_->getInternalIslandNodeIndex();
        physx_PxNodeIndex return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxArticulationLink_release(physx_PxArticulationLink* self__pod) {
        physx::PxArticulationLink* self_ = reinterpret_cast<physx::PxArticulationLink*>(self__pod);
        self_->release();
    }

    physx_PxArticulationReducedCoordinate* PxArticulationLink_getArticulation(physx_PxArticulationLink const* self__pod) {
        physx::PxArticulationLink const* self_ = reinterpret_cast<physx::PxArticulationLink const*>(self__pod);
        physx::PxArticulationReducedCoordinate& return_val = self_->getArticulation();
        auto return_val_pod = reinterpret_cast<physx_PxArticulationReducedCoordinate*>(&return_val);
        return return_val_pod;
    }

    physx_PxArticulationJointReducedCoordinate* PxArticulationLink_getInboundJoint(physx_PxArticulationLink const* self__pod) {
        physx::PxArticulationLink const* self_ = reinterpret_cast<physx::PxArticulationLink const*>(self__pod);
        physx::PxArticulationJointReducedCoordinate* return_val = self_->getInboundJoint();
        auto return_val_pod = reinterpret_cast<physx_PxArticulationJointReducedCoordinate*>(return_val);
        return return_val_pod;
    }

    uint32_t PxArticulationLink_getInboundJointDof(physx_PxArticulationLink const* self__pod) {
        physx::PxArticulationLink const* self_ = reinterpret_cast<physx::PxArticulationLink const*>(self__pod);
        uint32_t return_val = self_->getInboundJointDof();
        return return_val;
    }

    uint32_t PxArticulationLink_getNbChildren(physx_PxArticulationLink const* self__pod) {
        physx::PxArticulationLink const* self_ = reinterpret_cast<physx::PxArticulationLink const*>(self__pod);
        uint32_t return_val = self_->getNbChildren();
        return return_val;
    }

    uint32_t PxArticulationLink_getLinkIndex(physx_PxArticulationLink const* self__pod) {
        physx::PxArticulationLink const* self_ = reinterpret_cast<physx::PxArticulationLink const*>(self__pod);
        uint32_t return_val = self_->getLinkIndex();
        return return_val;
    }

    uint32_t PxArticulationLink_getChildren(physx_PxArticulationLink const* self__pod, physx_PxArticulationLink** userBuffer_pod, uint32_t bufferSize, uint32_t startIndex) {
        physx::PxArticulationLink const* self_ = reinterpret_cast<physx::PxArticulationLink const*>(self__pod);
        physx::PxArticulationLink** userBuffer = reinterpret_cast<physx::PxArticulationLink**>(userBuffer_pod);
        uint32_t return_val = self_->getChildren(userBuffer, bufferSize, startIndex);
        return return_val;
    }

    void PxArticulationLink_setCfmScale(physx_PxArticulationLink* self__pod, float cfm) {
        physx::PxArticulationLink* self_ = reinterpret_cast<physx::PxArticulationLink*>(self__pod);
        self_->setCfmScale(cfm);
    }

    float PxArticulationLink_getCfmScale(physx_PxArticulationLink const* self__pod) {
        physx::PxArticulationLink const* self_ = reinterpret_cast<physx::PxArticulationLink const*>(self__pod);
        float return_val = self_->getCfmScale();
        return return_val;
    }

    physx_PxVec3 PxArticulationLink_getLinearVelocity(physx_PxArticulationLink const* self__pod) {
        physx::PxArticulationLink const* self_ = reinterpret_cast<physx::PxArticulationLink const*>(self__pod);
        physx::PxVec3 return_val = self_->getLinearVelocity();
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3 PxArticulationLink_getAngularVelocity(physx_PxArticulationLink const* self__pod) {
        physx::PxArticulationLink const* self_ = reinterpret_cast<physx::PxArticulationLink const*>(self__pod);
        physx::PxVec3 return_val = self_->getAngularVelocity();
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    char const* PxArticulationLink_getConcreteTypeName(physx_PxArticulationLink const* self__pod) {
        physx::PxArticulationLink const* self_ = reinterpret_cast<physx::PxArticulationLink const*>(self__pod);
        char const* return_val = self_->getConcreteTypeName();
        return return_val;
    }

    physx_PxConeLimitedConstraint PxConeLimitedConstraint_new() {
        PxConeLimitedConstraint return_val;
        physx_PxConeLimitedConstraint return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxConstraint_release(physx_PxConstraint* self__pod) {
        physx::PxConstraint* self_ = reinterpret_cast<physx::PxConstraint*>(self__pod);
        self_->release();
    }

    physx_PxScene* PxConstraint_getScene(physx_PxConstraint const* self__pod) {
        physx::PxConstraint const* self_ = reinterpret_cast<physx::PxConstraint const*>(self__pod);
        physx::PxScene* return_val = self_->getScene();
        auto return_val_pod = reinterpret_cast<physx_PxScene*>(return_val);
        return return_val_pod;
    }

    void PxConstraint_getActors(physx_PxConstraint const* self__pod, physx_PxRigidActor** actor0_pod, physx_PxRigidActor** actor1_pod) {
        physx::PxConstraint const* self_ = reinterpret_cast<physx::PxConstraint const*>(self__pod);
        physx::PxRigidActor*& actor0 = reinterpret_cast<physx::PxRigidActor*&>(*actor0_pod);
        physx::PxRigidActor*& actor1 = reinterpret_cast<physx::PxRigidActor*&>(*actor1_pod);
        self_->getActors(actor0, actor1);
    }

    void PxConstraint_setActors(physx_PxConstraint* self__pod, physx_PxRigidActor* actor0_pod, physx_PxRigidActor* actor1_pod) {
        physx::PxConstraint* self_ = reinterpret_cast<physx::PxConstraint*>(self__pod);
        physx::PxRigidActor* actor0 = reinterpret_cast<physx::PxRigidActor*>(actor0_pod);
        physx::PxRigidActor* actor1 = reinterpret_cast<physx::PxRigidActor*>(actor1_pod);
        self_->setActors(actor0, actor1);
    }

    void PxConstraint_markDirty(physx_PxConstraint* self__pod) {
        physx::PxConstraint* self_ = reinterpret_cast<physx::PxConstraint*>(self__pod);
        self_->markDirty();
    }

    PxConstraintFlags PxConstraint_getFlags(physx_PxConstraint const* self__pod) {
        physx::PxConstraint const* self_ = reinterpret_cast<physx::PxConstraint const*>(self__pod);
        physx::PxConstraintFlags return_val = self_->getFlags();
        PxConstraintFlags return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxConstraint_setFlags(physx_PxConstraint* self__pod, PxConstraintFlags flags_pod) {
        physx::PxConstraint* self_ = reinterpret_cast<physx::PxConstraint*>(self__pod);
        auto flags = physx::PxConstraintFlags(flags_pod);
        self_->setFlags(flags);
    }

    void PxConstraint_setFlag(physx_PxConstraint* self__pod, PxConstraintFlag flag_pod, bool value) {
        physx::PxConstraint* self_ = reinterpret_cast<physx::PxConstraint*>(self__pod);
        auto flag = static_cast<physx::PxConstraintFlag::Enum>(flag_pod);
        self_->setFlag(flag, value);
    }

    void PxConstraint_getForce(physx_PxConstraint const* self__pod, physx_PxVec3* linear_pod, physx_PxVec3* angular_pod) {
        physx::PxConstraint const* self_ = reinterpret_cast<physx::PxConstraint const*>(self__pod);
        physx::PxVec3& linear = reinterpret_cast<physx::PxVec3&>(*linear_pod);
        physx::PxVec3& angular = reinterpret_cast<physx::PxVec3&>(*angular_pod);
        self_->getForce(linear, angular);
    }

    bool PxConstraint_isValid(physx_PxConstraint const* self__pod) {
        physx::PxConstraint const* self_ = reinterpret_cast<physx::PxConstraint const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    void PxConstraint_setBreakForce(physx_PxConstraint* self__pod, float linear, float angular) {
        physx::PxConstraint* self_ = reinterpret_cast<physx::PxConstraint*>(self__pod);
        self_->setBreakForce(linear, angular);
    }

    void PxConstraint_getBreakForce(physx_PxConstraint const* self__pod, float* linear_pod, float* angular_pod) {
        physx::PxConstraint const* self_ = reinterpret_cast<physx::PxConstraint const*>(self__pod);
        float& linear = *linear_pod;
        float& angular = *angular_pod;
        self_->getBreakForce(linear, angular);
    }

    void PxConstraint_setMinResponseThreshold(physx_PxConstraint* self__pod, float threshold) {
        physx::PxConstraint* self_ = reinterpret_cast<physx::PxConstraint*>(self__pod);
        self_->setMinResponseThreshold(threshold);
    }

    float PxConstraint_getMinResponseThreshold(physx_PxConstraint const* self__pod) {
        physx::PxConstraint const* self_ = reinterpret_cast<physx::PxConstraint const*>(self__pod);
        float return_val = self_->getMinResponseThreshold();
        return return_val;
    }

    void* PxConstraint_getExternalReference(physx_PxConstraint* self__pod, uint32_t* typeID_pod) {
        physx::PxConstraint* self_ = reinterpret_cast<physx::PxConstraint*>(self__pod);
        uint32_t& typeID = *typeID_pod;
        void* return_val = self_->getExternalReference(typeID);
        return return_val;
    }

    void PxConstraint_setConstraintFunctions(physx_PxConstraint* self__pod, physx_PxConstraintConnector* connector_pod, physx_PxConstraintShaderTable const* shaders_pod) {
        physx::PxConstraint* self_ = reinterpret_cast<physx::PxConstraint*>(self__pod);
        physx::PxConstraintConnector& connector = reinterpret_cast<physx::PxConstraintConnector&>(*connector_pod);
        physx::PxConstraintShaderTable const& shaders = reinterpret_cast<physx::PxConstraintShaderTable const&>(*shaders_pod);
        self_->setConstraintFunctions(connector, shaders);
    }

    char const* PxConstraint_getConcreteTypeName(physx_PxConstraint const* self__pod) {
        physx::PxConstraint const* self_ = reinterpret_cast<physx::PxConstraint const*>(self__pod);
        char const* return_val = self_->getConcreteTypeName();
        return return_val;
    }

    physx_PxContactStreamIterator PxContactStreamIterator_new(uint8_t const* contactPatches, uint8_t const* contactPoints, uint32_t const* contactFaceIndices, uint32_t nbPatches, uint32_t nbContacts) {
        PxContactStreamIterator return_val(contactPatches, contactPoints, contactFaceIndices, nbPatches, nbContacts);
        physx_PxContactStreamIterator return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxContactStreamIterator_hasNextPatch(physx_PxContactStreamIterator const* self__pod) {
        physx::PxContactStreamIterator const* self_ = reinterpret_cast<physx::PxContactStreamIterator const*>(self__pod);
        bool return_val = self_->hasNextPatch();
        return return_val;
    }

    uint32_t PxContactStreamIterator_getTotalContactCount(physx_PxContactStreamIterator const* self__pod) {
        physx::PxContactStreamIterator const* self_ = reinterpret_cast<physx::PxContactStreamIterator const*>(self__pod);
        uint32_t return_val = self_->getTotalContactCount();
        return return_val;
    }

    uint32_t PxContactStreamIterator_getTotalPatchCount(physx_PxContactStreamIterator const* self__pod) {
        physx::PxContactStreamIterator const* self_ = reinterpret_cast<physx::PxContactStreamIterator const*>(self__pod);
        uint32_t return_val = self_->getTotalPatchCount();
        return return_val;
    }

    void PxContactStreamIterator_nextPatch(physx_PxContactStreamIterator* self__pod) {
        physx::PxContactStreamIterator* self_ = reinterpret_cast<physx::PxContactStreamIterator*>(self__pod);
        self_->nextPatch();
    }

    bool PxContactStreamIterator_hasNextContact(physx_PxContactStreamIterator const* self__pod) {
        physx::PxContactStreamIterator const* self_ = reinterpret_cast<physx::PxContactStreamIterator const*>(self__pod);
        bool return_val = self_->hasNextContact();
        return return_val;
    }

    void PxContactStreamIterator_nextContact(physx_PxContactStreamIterator* self__pod) {
        physx::PxContactStreamIterator* self_ = reinterpret_cast<physx::PxContactStreamIterator*>(self__pod);
        self_->nextContact();
    }

    physx_PxVec3 const* PxContactStreamIterator_getContactNormal(physx_PxContactStreamIterator const* self__pod) {
        physx::PxContactStreamIterator const* self_ = reinterpret_cast<physx::PxContactStreamIterator const*>(self__pod);
        physx::PxVec3 const& return_val = self_->getContactNormal();
        auto return_val_pod = reinterpret_cast<physx_PxVec3 const*>(&return_val);
        return return_val_pod;
    }

    float PxContactStreamIterator_getInvMassScale0(physx_PxContactStreamIterator const* self__pod) {
        physx::PxContactStreamIterator const* self_ = reinterpret_cast<physx::PxContactStreamIterator const*>(self__pod);
        float return_val = self_->getInvMassScale0();
        return return_val;
    }

    float PxContactStreamIterator_getInvMassScale1(physx_PxContactStreamIterator const* self__pod) {
        physx::PxContactStreamIterator const* self_ = reinterpret_cast<physx::PxContactStreamIterator const*>(self__pod);
        float return_val = self_->getInvMassScale1();
        return return_val;
    }

    float PxContactStreamIterator_getInvInertiaScale0(physx_PxContactStreamIterator const* self__pod) {
        physx::PxContactStreamIterator const* self_ = reinterpret_cast<physx::PxContactStreamIterator const*>(self__pod);
        float return_val = self_->getInvInertiaScale0();
        return return_val;
    }

    float PxContactStreamIterator_getInvInertiaScale1(physx_PxContactStreamIterator const* self__pod) {
        physx::PxContactStreamIterator const* self_ = reinterpret_cast<physx::PxContactStreamIterator const*>(self__pod);
        float return_val = self_->getInvInertiaScale1();
        return return_val;
    }

    float PxContactStreamIterator_getMaxImpulse(physx_PxContactStreamIterator const* self__pod) {
        physx::PxContactStreamIterator const* self_ = reinterpret_cast<physx::PxContactStreamIterator const*>(self__pod);
        float return_val = self_->getMaxImpulse();
        return return_val;
    }

    physx_PxVec3 const* PxContactStreamIterator_getTargetVel(physx_PxContactStreamIterator const* self__pod) {
        physx::PxContactStreamIterator const* self_ = reinterpret_cast<physx::PxContactStreamIterator const*>(self__pod);
        physx::PxVec3 const& return_val = self_->getTargetVel();
        auto return_val_pod = reinterpret_cast<physx_PxVec3 const*>(&return_val);
        return return_val_pod;
    }

    physx_PxVec3 const* PxContactStreamIterator_getContactPoint(physx_PxContactStreamIterator const* self__pod) {
        physx::PxContactStreamIterator const* self_ = reinterpret_cast<physx::PxContactStreamIterator const*>(self__pod);
        physx::PxVec3 const& return_val = self_->getContactPoint();
        auto return_val_pod = reinterpret_cast<physx_PxVec3 const*>(&return_val);
        return return_val_pod;
    }

    float PxContactStreamIterator_getSeparation(physx_PxContactStreamIterator const* self__pod) {
        physx::PxContactStreamIterator const* self_ = reinterpret_cast<physx::PxContactStreamIterator const*>(self__pod);
        float return_val = self_->getSeparation();
        return return_val;
    }

    uint32_t PxContactStreamIterator_getFaceIndex0(physx_PxContactStreamIterator const* self__pod) {
        physx::PxContactStreamIterator const* self_ = reinterpret_cast<physx::PxContactStreamIterator const*>(self__pod);
        uint32_t return_val = self_->getFaceIndex0();
        return return_val;
    }

    uint32_t PxContactStreamIterator_getFaceIndex1(physx_PxContactStreamIterator const* self__pod) {
        physx::PxContactStreamIterator const* self_ = reinterpret_cast<physx::PxContactStreamIterator const*>(self__pod);
        uint32_t return_val = self_->getFaceIndex1();
        return return_val;
    }

    float PxContactStreamIterator_getStaticFriction(physx_PxContactStreamIterator const* self__pod) {
        physx::PxContactStreamIterator const* self_ = reinterpret_cast<physx::PxContactStreamIterator const*>(self__pod);
        float return_val = self_->getStaticFriction();
        return return_val;
    }

    float PxContactStreamIterator_getDynamicFriction(physx_PxContactStreamIterator const* self__pod) {
        physx::PxContactStreamIterator const* self_ = reinterpret_cast<physx::PxContactStreamIterator const*>(self__pod);
        float return_val = self_->getDynamicFriction();
        return return_val;
    }

    float PxContactStreamIterator_getRestitution(physx_PxContactStreamIterator const* self__pod) {
        physx::PxContactStreamIterator const* self_ = reinterpret_cast<physx::PxContactStreamIterator const*>(self__pod);
        float return_val = self_->getRestitution();
        return return_val;
    }

    float PxContactStreamIterator_getDamping(physx_PxContactStreamIterator const* self__pod) {
        physx::PxContactStreamIterator const* self_ = reinterpret_cast<physx::PxContactStreamIterator const*>(self__pod);
        float return_val = self_->getDamping();
        return return_val;
    }

    uint32_t PxContactStreamIterator_getMaterialFlags(physx_PxContactStreamIterator const* self__pod) {
        physx::PxContactStreamIterator const* self_ = reinterpret_cast<physx::PxContactStreamIterator const*>(self__pod);
        uint32_t return_val = self_->getMaterialFlags();
        return return_val;
    }

    uint16_t PxContactStreamIterator_getMaterialIndex0(physx_PxContactStreamIterator const* self__pod) {
        physx::PxContactStreamIterator const* self_ = reinterpret_cast<physx::PxContactStreamIterator const*>(self__pod);
        uint16_t return_val = self_->getMaterialIndex0();
        return return_val;
    }

    uint16_t PxContactStreamIterator_getMaterialIndex1(physx_PxContactStreamIterator const* self__pod) {
        physx::PxContactStreamIterator const* self_ = reinterpret_cast<physx::PxContactStreamIterator const*>(self__pod);
        uint16_t return_val = self_->getMaterialIndex1();
        return return_val;
    }

    bool PxContactStreamIterator_advanceToIndex(physx_PxContactStreamIterator* self__pod, uint32_t initialIndex) {
        physx::PxContactStreamIterator* self_ = reinterpret_cast<physx::PxContactStreamIterator*>(self__pod);
        bool return_val = self_->advanceToIndex(initialIndex);
        return return_val;
    }

    physx_PxVec3 const* PxContactSet_getPoint(physx_PxContactSet const* self__pod, uint32_t i) {
        physx::PxContactSet const* self_ = reinterpret_cast<physx::PxContactSet const*>(self__pod);
        physx::PxVec3 const& return_val = self_->getPoint(i);
        auto return_val_pod = reinterpret_cast<physx_PxVec3 const*>(&return_val);
        return return_val_pod;
    }

    void PxContactSet_setPoint(physx_PxContactSet* self__pod, uint32_t i, physx_PxVec3 const* p_pod) {
        physx::PxContactSet* self_ = reinterpret_cast<physx::PxContactSet*>(self__pod);
        physx::PxVec3 const& p = reinterpret_cast<physx::PxVec3 const&>(*p_pod);
        self_->setPoint(i, p);
    }

    physx_PxVec3 const* PxContactSet_getNormal(physx_PxContactSet const* self__pod, uint32_t i) {
        physx::PxContactSet const* self_ = reinterpret_cast<physx::PxContactSet const*>(self__pod);
        physx::PxVec3 const& return_val = self_->getNormal(i);
        auto return_val_pod = reinterpret_cast<physx_PxVec3 const*>(&return_val);
        return return_val_pod;
    }

    void PxContactSet_setNormal(physx_PxContactSet* self__pod, uint32_t i, physx_PxVec3 const* n_pod) {
        physx::PxContactSet* self_ = reinterpret_cast<physx::PxContactSet*>(self__pod);
        physx::PxVec3 const& n = reinterpret_cast<physx::PxVec3 const&>(*n_pod);
        self_->setNormal(i, n);
    }

    float PxContactSet_getSeparation(physx_PxContactSet const* self__pod, uint32_t i) {
        physx::PxContactSet const* self_ = reinterpret_cast<physx::PxContactSet const*>(self__pod);
        float return_val = self_->getSeparation(i);
        return return_val;
    }

    void PxContactSet_setSeparation(physx_PxContactSet* self__pod, uint32_t i, float s) {
        physx::PxContactSet* self_ = reinterpret_cast<physx::PxContactSet*>(self__pod);
        self_->setSeparation(i, s);
    }

    physx_PxVec3 const* PxContactSet_getTargetVelocity(physx_PxContactSet const* self__pod, uint32_t i) {
        physx::PxContactSet const* self_ = reinterpret_cast<physx::PxContactSet const*>(self__pod);
        physx::PxVec3 const& return_val = self_->getTargetVelocity(i);
        auto return_val_pod = reinterpret_cast<physx_PxVec3 const*>(&return_val);
        return return_val_pod;
    }

    void PxContactSet_setTargetVelocity(physx_PxContactSet* self__pod, uint32_t i, physx_PxVec3 const* v_pod) {
        physx::PxContactSet* self_ = reinterpret_cast<physx::PxContactSet*>(self__pod);
        physx::PxVec3 const& v = reinterpret_cast<physx::PxVec3 const&>(*v_pod);
        self_->setTargetVelocity(i, v);
    }

    uint32_t PxContactSet_getInternalFaceIndex0(physx_PxContactSet const* self__pod, uint32_t i) {
        physx::PxContactSet const* self_ = reinterpret_cast<physx::PxContactSet const*>(self__pod);
        uint32_t return_val = self_->getInternalFaceIndex0(i);
        return return_val;
    }

    uint32_t PxContactSet_getInternalFaceIndex1(physx_PxContactSet const* self__pod, uint32_t i) {
        physx::PxContactSet const* self_ = reinterpret_cast<physx::PxContactSet const*>(self__pod);
        uint32_t return_val = self_->getInternalFaceIndex1(i);
        return return_val;
    }

    float PxContactSet_getMaxImpulse(physx_PxContactSet const* self__pod, uint32_t i) {
        physx::PxContactSet const* self_ = reinterpret_cast<physx::PxContactSet const*>(self__pod);
        float return_val = self_->getMaxImpulse(i);
        return return_val;
    }

    void PxContactSet_setMaxImpulse(physx_PxContactSet* self__pod, uint32_t i, float s) {
        physx::PxContactSet* self_ = reinterpret_cast<physx::PxContactSet*>(self__pod);
        self_->setMaxImpulse(i, s);
    }

    float PxContactSet_getRestitution(physx_PxContactSet const* self__pod, uint32_t i) {
        physx::PxContactSet const* self_ = reinterpret_cast<physx::PxContactSet const*>(self__pod);
        float return_val = self_->getRestitution(i);
        return return_val;
    }

    void PxContactSet_setRestitution(physx_PxContactSet* self__pod, uint32_t i, float r) {
        physx::PxContactSet* self_ = reinterpret_cast<physx::PxContactSet*>(self__pod);
        self_->setRestitution(i, r);
    }

    float PxContactSet_getStaticFriction(physx_PxContactSet const* self__pod, uint32_t i) {
        physx::PxContactSet const* self_ = reinterpret_cast<physx::PxContactSet const*>(self__pod);
        float return_val = self_->getStaticFriction(i);
        return return_val;
    }

    void PxContactSet_setStaticFriction(physx_PxContactSet* self__pod, uint32_t i, float f) {
        physx::PxContactSet* self_ = reinterpret_cast<physx::PxContactSet*>(self__pod);
        self_->setStaticFriction(i, f);
    }

    float PxContactSet_getDynamicFriction(physx_PxContactSet const* self__pod, uint32_t i) {
        physx::PxContactSet const* self_ = reinterpret_cast<physx::PxContactSet const*>(self__pod);
        float return_val = self_->getDynamicFriction(i);
        return return_val;
    }

    void PxContactSet_setDynamicFriction(physx_PxContactSet* self__pod, uint32_t i, float f) {
        physx::PxContactSet* self_ = reinterpret_cast<physx::PxContactSet*>(self__pod);
        self_->setDynamicFriction(i, f);
    }

    void PxContactSet_ignore(physx_PxContactSet* self__pod, uint32_t i) {
        physx::PxContactSet* self_ = reinterpret_cast<physx::PxContactSet*>(self__pod);
        self_->ignore(i);
    }

    uint32_t PxContactSet_size(physx_PxContactSet const* self__pod) {
        physx::PxContactSet const* self_ = reinterpret_cast<physx::PxContactSet const*>(self__pod);
        uint32_t return_val = self_->size();
        return return_val;
    }

    float PxContactSet_getInvMassScale0(physx_PxContactSet const* self__pod) {
        physx::PxContactSet const* self_ = reinterpret_cast<physx::PxContactSet const*>(self__pod);
        float return_val = self_->getInvMassScale0();
        return return_val;
    }

    float PxContactSet_getInvMassScale1(physx_PxContactSet const* self__pod) {
        physx::PxContactSet const* self_ = reinterpret_cast<physx::PxContactSet const*>(self__pod);
        float return_val = self_->getInvMassScale1();
        return return_val;
    }

    float PxContactSet_getInvInertiaScale0(physx_PxContactSet const* self__pod) {
        physx::PxContactSet const* self_ = reinterpret_cast<physx::PxContactSet const*>(self__pod);
        float return_val = self_->getInvInertiaScale0();
        return return_val;
    }

    float PxContactSet_getInvInertiaScale1(physx_PxContactSet const* self__pod) {
        physx::PxContactSet const* self_ = reinterpret_cast<physx::PxContactSet const*>(self__pod);
        float return_val = self_->getInvInertiaScale1();
        return return_val;
    }

    void PxContactSet_setInvMassScale0(physx_PxContactSet* self__pod, float scale) {
        physx::PxContactSet* self_ = reinterpret_cast<physx::PxContactSet*>(self__pod);
        self_->setInvMassScale0(scale);
    }

    void PxContactSet_setInvMassScale1(physx_PxContactSet* self__pod, float scale) {
        physx::PxContactSet* self_ = reinterpret_cast<physx::PxContactSet*>(self__pod);
        self_->setInvMassScale1(scale);
    }

    void PxContactSet_setInvInertiaScale0(physx_PxContactSet* self__pod, float scale) {
        physx::PxContactSet* self_ = reinterpret_cast<physx::PxContactSet*>(self__pod);
        self_->setInvInertiaScale0(scale);
    }

    void PxContactSet_setInvInertiaScale1(physx_PxContactSet* self__pod, float scale) {
        physx::PxContactSet* self_ = reinterpret_cast<physx::PxContactSet*>(self__pod);
        self_->setInvInertiaScale1(scale);
    }

    void PxContactModifyCallback_onContactModify(physx_PxContactModifyCallback* self__pod, physx_PxContactModifyPair*const pairs_pod, uint32_t count) {
        physx::PxContactModifyCallback* self_ = reinterpret_cast<physx::PxContactModifyCallback*>(self__pod);
        physx::PxContactModifyPair*const pairs = reinterpret_cast<physx::PxContactModifyPair*const>(pairs_pod);
        self_->onContactModify(pairs, count);
    }

    void PxCCDContactModifyCallback_onCCDContactModify(physx_PxCCDContactModifyCallback* self__pod, physx_PxContactModifyPair*const pairs_pod, uint32_t count) {
        physx::PxCCDContactModifyCallback* self_ = reinterpret_cast<physx::PxCCDContactModifyCallback*>(self__pod);
        physx::PxContactModifyPair*const pairs = reinterpret_cast<physx::PxContactModifyPair*const>(pairs_pod);
        self_->onCCDContactModify(pairs, count);
    }

    void PxDeletionListener_onRelease(physx_PxDeletionListener* self__pod, physx_PxBase const* observed_pod, void* userData, PxDeletionEventFlag deletionEvent_pod) {
        physx::PxDeletionListener* self_ = reinterpret_cast<physx::PxDeletionListener*>(self__pod);
        physx::PxBase const* observed = reinterpret_cast<physx::PxBase const*>(observed_pod);
        auto deletionEvent = static_cast<physx::PxDeletionEventFlag::Enum>(deletionEvent_pod);
        self_->onRelease(observed, userData, deletionEvent);
    }

    bool PxBaseMaterial_isKindOf(physx_PxBaseMaterial const* self__pod, char const* name) {
        physx::PxBaseMaterial const* self_ = reinterpret_cast<physx::PxBaseMaterial const*>(self__pod);
        bool return_val = self_->isKindOf(name);
        return return_val;
    }

    void PxFEMMaterial_setYoungsModulus(physx_PxFEMMaterial* self__pod, float young) {
        physx::PxFEMMaterial* self_ = reinterpret_cast<physx::PxFEMMaterial*>(self__pod);
        self_->setYoungsModulus(young);
    }

    float PxFEMMaterial_getYoungsModulus(physx_PxFEMMaterial const* self__pod) {
        physx::PxFEMMaterial const* self_ = reinterpret_cast<physx::PxFEMMaterial const*>(self__pod);
        float return_val = self_->getYoungsModulus();
        return return_val;
    }

    void PxFEMMaterial_setPoissons(physx_PxFEMMaterial* self__pod, float poisson) {
        physx::PxFEMMaterial* self_ = reinterpret_cast<physx::PxFEMMaterial*>(self__pod);
        self_->setPoissons(poisson);
    }

    float PxFEMMaterial_getPoissons(physx_PxFEMMaterial const* self__pod) {
        physx::PxFEMMaterial const* self_ = reinterpret_cast<physx::PxFEMMaterial const*>(self__pod);
        float return_val = self_->getPoissons();
        return return_val;
    }

    void PxFEMMaterial_setDynamicFriction(physx_PxFEMMaterial* self__pod, float dynamicFriction) {
        physx::PxFEMMaterial* self_ = reinterpret_cast<physx::PxFEMMaterial*>(self__pod);
        self_->setDynamicFriction(dynamicFriction);
    }

    float PxFEMMaterial_getDynamicFriction(physx_PxFEMMaterial const* self__pod) {
        physx::PxFEMMaterial const* self_ = reinterpret_cast<physx::PxFEMMaterial const*>(self__pod);
        float return_val = self_->getDynamicFriction();
        return return_val;
    }

    physx_PxFilterData PxFilterData_new(PxEMPTY anon_param0_pod) {
        auto anon_param0 = static_cast<physx::PxEMPTY>(anon_param0_pod);
        PxFilterData return_val(anon_param0);
        physx_PxFilterData return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxFilterData PxFilterData_new_1() {
        PxFilterData return_val;
        physx_PxFilterData return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxFilterData PxFilterData_new_2(uint32_t w0, uint32_t w1, uint32_t w2, uint32_t w3) {
        PxFilterData return_val(w0, w1, w2, w3);
        physx_PxFilterData return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxFilterData_setToDefault(physx_PxFilterData* self__pod) {
        physx::PxFilterData* self_ = reinterpret_cast<physx::PxFilterData*>(self__pod);
        self_->setToDefault();
    }

    PxFilterObjectType phys_PxGetFilterObjectType(uint32_t attr) {
        physx::PxFilterObjectType::Enum return_val = PxGetFilterObjectType(attr);
        PxFilterObjectType return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool phys_PxFilterObjectIsKinematic(uint32_t attr) {
        bool return_val = PxFilterObjectIsKinematic(attr);
        return return_val;
    }

    bool phys_PxFilterObjectIsTrigger(uint32_t attr) {
        bool return_val = PxFilterObjectIsTrigger(attr);
        return return_val;
    }

    PxFilterFlags PxSimulationFilterCallback_pairFound(physx_PxSimulationFilterCallback* self__pod, uint32_t pairID, uint32_t attributes0, physx_PxFilterData filterData0_pod, physx_PxActor const* a0_pod, physx_PxShape const* s0_pod, uint32_t attributes1, physx_PxFilterData filterData1_pod, physx_PxActor const* a1_pod, physx_PxShape const* s1_pod, PxPairFlags* pairFlags_pod) {
        physx::PxSimulationFilterCallback* self_ = reinterpret_cast<physx::PxSimulationFilterCallback*>(self__pod);
        physx::PxFilterData filterData0;
        memcpy(&filterData0, &filterData0_pod, sizeof(filterData0));
        physx::PxActor const* a0 = reinterpret_cast<physx::PxActor const*>(a0_pod);
        physx::PxShape const* s0 = reinterpret_cast<physx::PxShape const*>(s0_pod);
        physx::PxFilterData filterData1;
        memcpy(&filterData1, &filterData1_pod, sizeof(filterData1));
        physx::PxActor const* a1 = reinterpret_cast<physx::PxActor const*>(a1_pod);
        physx::PxShape const* s1 = reinterpret_cast<physx::PxShape const*>(s1_pod);
        physx::PxPairFlags& pairFlags = reinterpret_cast<physx::PxPairFlags&>(*pairFlags_pod);
        physx::PxFilterFlags return_val = self_->pairFound(pairID, attributes0, filterData0, a0, s0, attributes1, filterData1, a1, s1, pairFlags);
        PxFilterFlags return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxSimulationFilterCallback_pairLost(physx_PxSimulationFilterCallback* self__pod, uint32_t pairID, uint32_t attributes0, physx_PxFilterData filterData0_pod, uint32_t attributes1, physx_PxFilterData filterData1_pod, bool objectRemoved) {
        physx::PxSimulationFilterCallback* self_ = reinterpret_cast<physx::PxSimulationFilterCallback*>(self__pod);
        physx::PxFilterData filterData0;
        memcpy(&filterData0, &filterData0_pod, sizeof(filterData0));
        physx::PxFilterData filterData1;
        memcpy(&filterData1, &filterData1_pod, sizeof(filterData1));
        self_->pairLost(pairID, attributes0, filterData0, attributes1, filterData1, objectRemoved);
    }

    bool PxSimulationFilterCallback_statusChange(physx_PxSimulationFilterCallback* self__pod, uint32_t* pairID_pod, PxPairFlags* pairFlags_pod, PxFilterFlags* filterFlags_pod) {
        physx::PxSimulationFilterCallback* self_ = reinterpret_cast<physx::PxSimulationFilterCallback*>(self__pod);
        uint32_t& pairID = *pairID_pod;
        physx::PxPairFlags& pairFlags = reinterpret_cast<physx::PxPairFlags&>(*pairFlags_pod);
        physx::PxFilterFlags& filterFlags = reinterpret_cast<physx::PxFilterFlags&>(*filterFlags_pod);
        bool return_val = self_->statusChange(pairID, pairFlags, filterFlags);
        return return_val;
    }

    PxDataAccessFlags PxLockedData_getDataAccessFlags(physx_PxLockedData* self__pod) {
        physx::PxLockedData* self_ = reinterpret_cast<physx::PxLockedData*>(self__pod);
        physx::PxDataAccessFlags return_val = self_->getDataAccessFlags();
        PxDataAccessFlags return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxLockedData_unlock(physx_PxLockedData* self__pod) {
        physx::PxLockedData* self_ = reinterpret_cast<physx::PxLockedData*>(self__pod);
        self_->unlock();
    }

    void PxLockedData_delete(physx_PxLockedData* self__pod) {
        physx::PxLockedData* self_ = reinterpret_cast<physx::PxLockedData*>(self__pod);
        delete self_;
    }

    void PxMaterial_setDynamicFriction(physx_PxMaterial* self__pod, float coef) {
        physx::PxMaterial* self_ = reinterpret_cast<physx::PxMaterial*>(self__pod);
        self_->setDynamicFriction(coef);
    }

    float PxMaterial_getDynamicFriction(physx_PxMaterial const* self__pod) {
        physx::PxMaterial const* self_ = reinterpret_cast<physx::PxMaterial const*>(self__pod);
        float return_val = self_->getDynamicFriction();
        return return_val;
    }

    void PxMaterial_setStaticFriction(physx_PxMaterial* self__pod, float coef) {
        physx::PxMaterial* self_ = reinterpret_cast<physx::PxMaterial*>(self__pod);
        self_->setStaticFriction(coef);
    }

    float PxMaterial_getStaticFriction(physx_PxMaterial const* self__pod) {
        physx::PxMaterial const* self_ = reinterpret_cast<physx::PxMaterial const*>(self__pod);
        float return_val = self_->getStaticFriction();
        return return_val;
    }

    void PxMaterial_setRestitution(physx_PxMaterial* self__pod, float rest) {
        physx::PxMaterial* self_ = reinterpret_cast<physx::PxMaterial*>(self__pod);
        self_->setRestitution(rest);
    }

    float PxMaterial_getRestitution(physx_PxMaterial const* self__pod) {
        physx::PxMaterial const* self_ = reinterpret_cast<physx::PxMaterial const*>(self__pod);
        float return_val = self_->getRestitution();
        return return_val;
    }

    void PxMaterial_setDamping(physx_PxMaterial* self__pod, float damping) {
        physx::PxMaterial* self_ = reinterpret_cast<physx::PxMaterial*>(self__pod);
        self_->setDamping(damping);
    }

    float PxMaterial_getDamping(physx_PxMaterial const* self__pod) {
        physx::PxMaterial const* self_ = reinterpret_cast<physx::PxMaterial const*>(self__pod);
        float return_val = self_->getDamping();
        return return_val;
    }

    void PxMaterial_setFlag(physx_PxMaterial* self__pod, PxMaterialFlag flag_pod, bool b) {
        physx::PxMaterial* self_ = reinterpret_cast<physx::PxMaterial*>(self__pod);
        auto flag = static_cast<physx::PxMaterialFlag::Enum>(flag_pod);
        self_->setFlag(flag, b);
    }

    void PxMaterial_setFlags(physx_PxMaterial* self__pod, PxMaterialFlags flags_pod) {
        physx::PxMaterial* self_ = reinterpret_cast<physx::PxMaterial*>(self__pod);
        auto flags = physx::PxMaterialFlags(flags_pod);
        self_->setFlags(flags);
    }

    PxMaterialFlags PxMaterial_getFlags(physx_PxMaterial const* self__pod) {
        physx::PxMaterial const* self_ = reinterpret_cast<physx::PxMaterial const*>(self__pod);
        physx::PxMaterialFlags return_val = self_->getFlags();
        PxMaterialFlags return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxMaterial_setFrictionCombineMode(physx_PxMaterial* self__pod, PxCombineMode combMode_pod) {
        physx::PxMaterial* self_ = reinterpret_cast<physx::PxMaterial*>(self__pod);
        auto combMode = static_cast<physx::PxCombineMode::Enum>(combMode_pod);
        self_->setFrictionCombineMode(combMode);
    }

    PxCombineMode PxMaterial_getFrictionCombineMode(physx_PxMaterial const* self__pod) {
        physx::PxMaterial const* self_ = reinterpret_cast<physx::PxMaterial const*>(self__pod);
        physx::PxCombineMode::Enum return_val = self_->getFrictionCombineMode();
        PxCombineMode return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxMaterial_setRestitutionCombineMode(physx_PxMaterial* self__pod, PxCombineMode combMode_pod) {
        physx::PxMaterial* self_ = reinterpret_cast<physx::PxMaterial*>(self__pod);
        auto combMode = static_cast<physx::PxCombineMode::Enum>(combMode_pod);
        self_->setRestitutionCombineMode(combMode);
    }

    PxCombineMode PxMaterial_getRestitutionCombineMode(physx_PxMaterial const* self__pod) {
        physx::PxMaterial const* self_ = reinterpret_cast<physx::PxMaterial const*>(self__pod);
        physx::PxCombineMode::Enum return_val = self_->getRestitutionCombineMode();
        PxCombineMode return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    char const* PxMaterial_getConcreteTypeName(physx_PxMaterial const* self__pod) {
        physx::PxMaterial const* self_ = reinterpret_cast<physx::PxMaterial const*>(self__pod);
        char const* return_val = self_->getConcreteTypeName();
        return return_val;
    }

    physx_PxDiffuseParticleParams PxDiffuseParticleParams_new() {
        PxDiffuseParticleParams return_val;
        physx_PxDiffuseParticleParams return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxDiffuseParticleParams_setToDefault(physx_PxDiffuseParticleParams* self__pod) {
        physx::PxDiffuseParticleParams* self_ = reinterpret_cast<physx::PxDiffuseParticleParams*>(self__pod);
        self_->setToDefault();
    }

    void PxParticleMaterial_setFriction(physx_PxParticleMaterial* self__pod, float friction) {
        physx::PxParticleMaterial* self_ = reinterpret_cast<physx::PxParticleMaterial*>(self__pod);
        self_->setFriction(friction);
    }

    float PxParticleMaterial_getFriction(physx_PxParticleMaterial const* self__pod) {
        physx::PxParticleMaterial const* self_ = reinterpret_cast<physx::PxParticleMaterial const*>(self__pod);
        float return_val = self_->getFriction();
        return return_val;
    }

    void PxParticleMaterial_setDamping(physx_PxParticleMaterial* self__pod, float damping) {
        physx::PxParticleMaterial* self_ = reinterpret_cast<physx::PxParticleMaterial*>(self__pod);
        self_->setDamping(damping);
    }

    float PxParticleMaterial_getDamping(physx_PxParticleMaterial const* self__pod) {
        physx::PxParticleMaterial const* self_ = reinterpret_cast<physx::PxParticleMaterial const*>(self__pod);
        float return_val = self_->getDamping();
        return return_val;
    }

    void PxParticleMaterial_setAdhesion(physx_PxParticleMaterial* self__pod, float adhesion) {
        physx::PxParticleMaterial* self_ = reinterpret_cast<physx::PxParticleMaterial*>(self__pod);
        self_->setAdhesion(adhesion);
    }

    float PxParticleMaterial_getAdhesion(physx_PxParticleMaterial const* self__pod) {
        physx::PxParticleMaterial const* self_ = reinterpret_cast<physx::PxParticleMaterial const*>(self__pod);
        float return_val = self_->getAdhesion();
        return return_val;
    }

    void PxParticleMaterial_setGravityScale(physx_PxParticleMaterial* self__pod, float scale) {
        physx::PxParticleMaterial* self_ = reinterpret_cast<physx::PxParticleMaterial*>(self__pod);
        self_->setGravityScale(scale);
    }

    float PxParticleMaterial_getGravityScale(physx_PxParticleMaterial const* self__pod) {
        physx::PxParticleMaterial const* self_ = reinterpret_cast<physx::PxParticleMaterial const*>(self__pod);
        float return_val = self_->getGravityScale();
        return return_val;
    }

    void PxParticleMaterial_setAdhesionRadiusScale(physx_PxParticleMaterial* self__pod, float scale) {
        physx::PxParticleMaterial* self_ = reinterpret_cast<physx::PxParticleMaterial*>(self__pod);
        self_->setAdhesionRadiusScale(scale);
    }

    float PxParticleMaterial_getAdhesionRadiusScale(physx_PxParticleMaterial const* self__pod) {
        physx::PxParticleMaterial const* self_ = reinterpret_cast<physx::PxParticleMaterial const*>(self__pod);
        float return_val = self_->getAdhesionRadiusScale();
        return return_val;
    }

    void PxPhysics_release(physx_PxPhysics* self__pod) {
        physx::PxPhysics* self_ = reinterpret_cast<physx::PxPhysics*>(self__pod);
        self_->release();
    }

    physx_PxFoundation* PxPhysics_getFoundation(physx_PxPhysics* self__pod) {
        physx::PxPhysics* self_ = reinterpret_cast<physx::PxPhysics*>(self__pod);
        physx::PxFoundation& return_val = self_->getFoundation();
        auto return_val_pod = reinterpret_cast<physx_PxFoundation*>(&return_val);
        return return_val_pod;
    }

    physx_PxAggregate* PxPhysics_createAggregate(physx_PxPhysics* self__pod, uint32_t maxActor, uint32_t maxShape, uint32_t filterHint) {
        physx::PxPhysics* self_ = reinterpret_cast<physx::PxPhysics*>(self__pod);
        physx::PxAggregate* return_val = self_->createAggregate(maxActor, maxShape, filterHint);
        auto return_val_pod = reinterpret_cast<physx_PxAggregate*>(return_val);
        return return_val_pod;
    }

    physx_PxTolerancesScale const* PxPhysics_getTolerancesScale(physx_PxPhysics const* self__pod) {
        physx::PxPhysics const* self_ = reinterpret_cast<physx::PxPhysics const*>(self__pod);
        physx::PxTolerancesScale const& return_val = self_->getTolerancesScale();
        auto return_val_pod = reinterpret_cast<physx_PxTolerancesScale const*>(&return_val);
        return return_val_pod;
    }

    physx_PxTriangleMesh* PxPhysics_createTriangleMesh(physx_PxPhysics* self__pod, physx_PxInputStream* stream_pod) {
        physx::PxPhysics* self_ = reinterpret_cast<physx::PxPhysics*>(self__pod);
        physx::PxInputStream& stream = reinterpret_cast<physx::PxInputStream&>(*stream_pod);
        physx::PxTriangleMesh* return_val = self_->createTriangleMesh(stream);
        auto return_val_pod = reinterpret_cast<physx_PxTriangleMesh*>(return_val);
        return return_val_pod;
    }

    uint32_t PxPhysics_getNbTriangleMeshes(physx_PxPhysics const* self__pod) {
        physx::PxPhysics const* self_ = reinterpret_cast<physx::PxPhysics const*>(self__pod);
        uint32_t return_val = self_->getNbTriangleMeshes();
        return return_val;
    }

    uint32_t PxPhysics_getTriangleMeshes(physx_PxPhysics const* self__pod, physx_PxTriangleMesh** userBuffer_pod, uint32_t bufferSize, uint32_t startIndex) {
        physx::PxPhysics const* self_ = reinterpret_cast<physx::PxPhysics const*>(self__pod);
        physx::PxTriangleMesh** userBuffer = reinterpret_cast<physx::PxTriangleMesh**>(userBuffer_pod);
        uint32_t return_val = self_->getTriangleMeshes(userBuffer, bufferSize, startIndex);
        return return_val;
    }

    physx_PxTetrahedronMesh* PxPhysics_createTetrahedronMesh(physx_PxPhysics* self__pod, physx_PxInputStream* stream_pod) {
        physx::PxPhysics* self_ = reinterpret_cast<physx::PxPhysics*>(self__pod);
        physx::PxInputStream& stream = reinterpret_cast<physx::PxInputStream&>(*stream_pod);
        physx::PxTetrahedronMesh* return_val = self_->createTetrahedronMesh(stream);
        auto return_val_pod = reinterpret_cast<physx_PxTetrahedronMesh*>(return_val);
        return return_val_pod;
    }

    physx_PxSoftBodyMesh* PxPhysics_createSoftBodyMesh(physx_PxPhysics* self__pod, physx_PxInputStream* stream_pod) {
        physx::PxPhysics* self_ = reinterpret_cast<physx::PxPhysics*>(self__pod);
        physx::PxInputStream& stream = reinterpret_cast<physx::PxInputStream&>(*stream_pod);
        physx::PxSoftBodyMesh* return_val = self_->createSoftBodyMesh(stream);
        auto return_val_pod = reinterpret_cast<physx_PxSoftBodyMesh*>(return_val);
        return return_val_pod;
    }

    uint32_t PxPhysics_getNbTetrahedronMeshes(physx_PxPhysics const* self__pod) {
        physx::PxPhysics const* self_ = reinterpret_cast<physx::PxPhysics const*>(self__pod);
        uint32_t return_val = self_->getNbTetrahedronMeshes();
        return return_val;
    }

    uint32_t PxPhysics_getTetrahedronMeshes(physx_PxPhysics const* self__pod, physx_PxTetrahedronMesh** userBuffer_pod, uint32_t bufferSize, uint32_t startIndex) {
        physx::PxPhysics const* self_ = reinterpret_cast<physx::PxPhysics const*>(self__pod);
        physx::PxTetrahedronMesh** userBuffer = reinterpret_cast<physx::PxTetrahedronMesh**>(userBuffer_pod);
        uint32_t return_val = self_->getTetrahedronMeshes(userBuffer, bufferSize, startIndex);
        return return_val;
    }

    physx_PxHeightField* PxPhysics_createHeightField(physx_PxPhysics* self__pod, physx_PxInputStream* stream_pod) {
        physx::PxPhysics* self_ = reinterpret_cast<physx::PxPhysics*>(self__pod);
        physx::PxInputStream& stream = reinterpret_cast<physx::PxInputStream&>(*stream_pod);
        physx::PxHeightField* return_val = self_->createHeightField(stream);
        auto return_val_pod = reinterpret_cast<physx_PxHeightField*>(return_val);
        return return_val_pod;
    }

    uint32_t PxPhysics_getNbHeightFields(physx_PxPhysics const* self__pod) {
        physx::PxPhysics const* self_ = reinterpret_cast<physx::PxPhysics const*>(self__pod);
        uint32_t return_val = self_->getNbHeightFields();
        return return_val;
    }

    uint32_t PxPhysics_getHeightFields(physx_PxPhysics const* self__pod, physx_PxHeightField** userBuffer_pod, uint32_t bufferSize, uint32_t startIndex) {
        physx::PxPhysics const* self_ = reinterpret_cast<physx::PxPhysics const*>(self__pod);
        physx::PxHeightField** userBuffer = reinterpret_cast<physx::PxHeightField**>(userBuffer_pod);
        uint32_t return_val = self_->getHeightFields(userBuffer, bufferSize, startIndex);
        return return_val;
    }

    physx_PxConvexMesh* PxPhysics_createConvexMesh(physx_PxPhysics* self__pod, physx_PxInputStream* stream_pod) {
        physx::PxPhysics* self_ = reinterpret_cast<physx::PxPhysics*>(self__pod);
        physx::PxInputStream& stream = reinterpret_cast<physx::PxInputStream&>(*stream_pod);
        physx::PxConvexMesh* return_val = self_->createConvexMesh(stream);
        auto return_val_pod = reinterpret_cast<physx_PxConvexMesh*>(return_val);
        return return_val_pod;
    }

    uint32_t PxPhysics_getNbConvexMeshes(physx_PxPhysics const* self__pod) {
        physx::PxPhysics const* self_ = reinterpret_cast<physx::PxPhysics const*>(self__pod);
        uint32_t return_val = self_->getNbConvexMeshes();
        return return_val;
    }

    uint32_t PxPhysics_getConvexMeshes(physx_PxPhysics const* self__pod, physx_PxConvexMesh** userBuffer_pod, uint32_t bufferSize, uint32_t startIndex) {
        physx::PxPhysics const* self_ = reinterpret_cast<physx::PxPhysics const*>(self__pod);
        physx::PxConvexMesh** userBuffer = reinterpret_cast<physx::PxConvexMesh**>(userBuffer_pod);
        uint32_t return_val = self_->getConvexMeshes(userBuffer, bufferSize, startIndex);
        return return_val;
    }

    physx_PxBVH* PxPhysics_createBVH(physx_PxPhysics* self__pod, physx_PxInputStream* stream_pod) {
        physx::PxPhysics* self_ = reinterpret_cast<physx::PxPhysics*>(self__pod);
        physx::PxInputStream& stream = reinterpret_cast<physx::PxInputStream&>(*stream_pod);
        physx::PxBVH* return_val = self_->createBVH(stream);
        auto return_val_pod = reinterpret_cast<physx_PxBVH*>(return_val);
        return return_val_pod;
    }

    uint32_t PxPhysics_getNbBVHs(physx_PxPhysics const* self__pod) {
        physx::PxPhysics const* self_ = reinterpret_cast<physx::PxPhysics const*>(self__pod);
        uint32_t return_val = self_->getNbBVHs();
        return return_val;
    }

    uint32_t PxPhysics_getBVHs(physx_PxPhysics const* self__pod, physx_PxBVH** userBuffer_pod, uint32_t bufferSize, uint32_t startIndex) {
        physx::PxPhysics const* self_ = reinterpret_cast<physx::PxPhysics const*>(self__pod);
        physx::PxBVH** userBuffer = reinterpret_cast<physx::PxBVH**>(userBuffer_pod);
        uint32_t return_val = self_->getBVHs(userBuffer, bufferSize, startIndex);
        return return_val;
    }

    physx_PxScene* PxPhysics_createScene(physx_PxPhysics* self__pod, physx_PxSceneDesc const* sceneDesc_pod) {
        physx::PxPhysics* self_ = reinterpret_cast<physx::PxPhysics*>(self__pod);
        physx::PxSceneDesc const& sceneDesc = reinterpret_cast<physx::PxSceneDesc const&>(*sceneDesc_pod);
        physx::PxScene* return_val = self_->createScene(sceneDesc);
        auto return_val_pod = reinterpret_cast<physx_PxScene*>(return_val);
        return return_val_pod;
    }

    uint32_t PxPhysics_getNbScenes(physx_PxPhysics const* self__pod) {
        physx::PxPhysics const* self_ = reinterpret_cast<physx::PxPhysics const*>(self__pod);
        uint32_t return_val = self_->getNbScenes();
        return return_val;
    }

    uint32_t PxPhysics_getScenes(physx_PxPhysics const* self__pod, physx_PxScene** userBuffer_pod, uint32_t bufferSize, uint32_t startIndex) {
        physx::PxPhysics const* self_ = reinterpret_cast<physx::PxPhysics const*>(self__pod);
        physx::PxScene** userBuffer = reinterpret_cast<physx::PxScene**>(userBuffer_pod);
        uint32_t return_val = self_->getScenes(userBuffer, bufferSize, startIndex);
        return return_val;
    }

    physx_PxRigidStatic* PxPhysics_createRigidStatic(physx_PxPhysics* self__pod, physx_PxTransform const* pose_pod) {
        physx::PxPhysics* self_ = reinterpret_cast<physx::PxPhysics*>(self__pod);
        physx::PxTransform const& pose = reinterpret_cast<physx::PxTransform const&>(*pose_pod);
        physx::PxRigidStatic* return_val = self_->createRigidStatic(pose);
        auto return_val_pod = reinterpret_cast<physx_PxRigidStatic*>(return_val);
        return return_val_pod;
    }

    physx_PxRigidDynamic* PxPhysics_createRigidDynamic(physx_PxPhysics* self__pod, physx_PxTransform const* pose_pod) {
        physx::PxPhysics* self_ = reinterpret_cast<physx::PxPhysics*>(self__pod);
        physx::PxTransform const& pose = reinterpret_cast<physx::PxTransform const&>(*pose_pod);
        physx::PxRigidDynamic* return_val = self_->createRigidDynamic(pose);
        auto return_val_pod = reinterpret_cast<physx_PxRigidDynamic*>(return_val);
        return return_val_pod;
    }

    physx_PxPruningStructure* PxPhysics_createPruningStructure(physx_PxPhysics* self__pod, physx_PxRigidActor* const* actors_pod, uint32_t nbActors) {
        physx::PxPhysics* self_ = reinterpret_cast<physx::PxPhysics*>(self__pod);
        physx::PxRigidActor* const* actors = reinterpret_cast<physx::PxRigidActor* const*>(actors_pod);
        physx::PxPruningStructure* return_val = self_->createPruningStructure(actors, nbActors);
        auto return_val_pod = reinterpret_cast<physx_PxPruningStructure*>(return_val);
        return return_val_pod;
    }

    physx_PxShape* PxPhysics_createShape(physx_PxPhysics* self__pod, physx_PxGeometry const* geometry_pod, physx_PxMaterial const* material_pod, bool isExclusive, PxShapeFlags shapeFlags_pod) {
        physx::PxPhysics* self_ = reinterpret_cast<physx::PxPhysics*>(self__pod);
        physx::PxGeometry const& geometry = reinterpret_cast<physx::PxGeometry const&>(*geometry_pod);
        physx::PxMaterial const& material = reinterpret_cast<physx::PxMaterial const&>(*material_pod);
        auto shapeFlags = physx::PxShapeFlags(shapeFlags_pod);
        physx::PxShape* return_val = self_->createShape(geometry, material, isExclusive, shapeFlags);
        auto return_val_pod = reinterpret_cast<physx_PxShape*>(return_val);
        return return_val_pod;
    }

    physx_PxShape* PxPhysics_createShape_1(physx_PxPhysics* self__pod, physx_PxGeometry const* geometry_pod, physx_PxMaterial* const* materials_pod, uint16_t materialCount, bool isExclusive, PxShapeFlags shapeFlags_pod) {
        physx::PxPhysics* self_ = reinterpret_cast<physx::PxPhysics*>(self__pod);
        physx::PxGeometry const& geometry = reinterpret_cast<physx::PxGeometry const&>(*geometry_pod);
        physx::PxMaterial* const* materials = reinterpret_cast<physx::PxMaterial* const*>(materials_pod);
        auto shapeFlags = physx::PxShapeFlags(shapeFlags_pod);
        physx::PxShape* return_val = self_->createShape(geometry, materials, materialCount, isExclusive, shapeFlags);
        auto return_val_pod = reinterpret_cast<physx_PxShape*>(return_val);
        return return_val_pod;
    }

    uint32_t PxPhysics_getNbShapes(physx_PxPhysics const* self__pod) {
        physx::PxPhysics const* self_ = reinterpret_cast<physx::PxPhysics const*>(self__pod);
        uint32_t return_val = self_->getNbShapes();
        return return_val;
    }

    uint32_t PxPhysics_getShapes(physx_PxPhysics const* self__pod, physx_PxShape** userBuffer_pod, uint32_t bufferSize, uint32_t startIndex) {
        physx::PxPhysics const* self_ = reinterpret_cast<physx::PxPhysics const*>(self__pod);
        physx::PxShape** userBuffer = reinterpret_cast<physx::PxShape**>(userBuffer_pod);
        uint32_t return_val = self_->getShapes(userBuffer, bufferSize, startIndex);
        return return_val;
    }

    physx_PxConstraint* PxPhysics_createConstraint(physx_PxPhysics* self__pod, physx_PxRigidActor* actor0_pod, physx_PxRigidActor* actor1_pod, physx_PxConstraintConnector* connector_pod, physx_PxConstraintShaderTable const* shaders_pod, uint32_t dataSize) {
        physx::PxPhysics* self_ = reinterpret_cast<physx::PxPhysics*>(self__pod);
        physx::PxRigidActor* actor0 = reinterpret_cast<physx::PxRigidActor*>(actor0_pod);
        physx::PxRigidActor* actor1 = reinterpret_cast<physx::PxRigidActor*>(actor1_pod);
        physx::PxConstraintConnector& connector = reinterpret_cast<physx::PxConstraintConnector&>(*connector_pod);
        physx::PxConstraintShaderTable const& shaders = reinterpret_cast<physx::PxConstraintShaderTable const&>(*shaders_pod);
        physx::PxConstraint* return_val = self_->createConstraint(actor0, actor1, connector, shaders, dataSize);
        auto return_val_pod = reinterpret_cast<physx_PxConstraint*>(return_val);
        return return_val_pod;
    }

    physx_PxArticulationReducedCoordinate* PxPhysics_createArticulationReducedCoordinate(physx_PxPhysics* self__pod) {
        physx::PxPhysics* self_ = reinterpret_cast<physx::PxPhysics*>(self__pod);
        physx::PxArticulationReducedCoordinate* return_val = self_->createArticulationReducedCoordinate();
        auto return_val_pod = reinterpret_cast<physx_PxArticulationReducedCoordinate*>(return_val);
        return return_val_pod;
    }

    physx_PxMaterial* PxPhysics_createMaterial(physx_PxPhysics* self__pod, float staticFriction, float dynamicFriction, float restitution) {
        physx::PxPhysics* self_ = reinterpret_cast<physx::PxPhysics*>(self__pod);
        physx::PxMaterial* return_val = self_->createMaterial(staticFriction, dynamicFriction, restitution);
        auto return_val_pod = reinterpret_cast<physx_PxMaterial*>(return_val);
        return return_val_pod;
    }

    uint32_t PxPhysics_getNbMaterials(physx_PxPhysics const* self__pod) {
        physx::PxPhysics const* self_ = reinterpret_cast<physx::PxPhysics const*>(self__pod);
        uint32_t return_val = self_->getNbMaterials();
        return return_val;
    }

    uint32_t PxPhysics_getMaterials(physx_PxPhysics const* self__pod, physx_PxMaterial** userBuffer_pod, uint32_t bufferSize, uint32_t startIndex) {
        physx::PxPhysics const* self_ = reinterpret_cast<physx::PxPhysics const*>(self__pod);
        physx::PxMaterial** userBuffer = reinterpret_cast<physx::PxMaterial**>(userBuffer_pod);
        uint32_t return_val = self_->getMaterials(userBuffer, bufferSize, startIndex);
        return return_val;
    }

    void PxPhysics_registerDeletionListener(physx_PxPhysics* self__pod, physx_PxDeletionListener* observer_pod, PxDeletionEventFlags const* deletionEvents_pod, bool restrictedObjectSet) {
        physx::PxPhysics* self_ = reinterpret_cast<physx::PxPhysics*>(self__pod);
        physx::PxDeletionListener& observer = reinterpret_cast<physx::PxDeletionListener&>(*observer_pod);
        physx::PxDeletionEventFlags const& deletionEvents = reinterpret_cast<physx::PxDeletionEventFlags const&>(*deletionEvents_pod);
        self_->registerDeletionListener(observer, deletionEvents, restrictedObjectSet);
    }

    void PxPhysics_unregisterDeletionListener(physx_PxPhysics* self__pod, physx_PxDeletionListener* observer_pod) {
        physx::PxPhysics* self_ = reinterpret_cast<physx::PxPhysics*>(self__pod);
        physx::PxDeletionListener& observer = reinterpret_cast<physx::PxDeletionListener&>(*observer_pod);
        self_->unregisterDeletionListener(observer);
    }

    void PxPhysics_registerDeletionListenerObjects(physx_PxPhysics* self__pod, physx_PxDeletionListener* observer_pod, physx_PxBase const* const* observables_pod, uint32_t observableCount) {
        physx::PxPhysics* self_ = reinterpret_cast<physx::PxPhysics*>(self__pod);
        physx::PxDeletionListener& observer = reinterpret_cast<physx::PxDeletionListener&>(*observer_pod);
        physx::PxBase const* const* observables = reinterpret_cast<physx::PxBase const* const*>(observables_pod);
        self_->registerDeletionListenerObjects(observer, observables, observableCount);
    }

    void PxPhysics_unregisterDeletionListenerObjects(physx_PxPhysics* self__pod, physx_PxDeletionListener* observer_pod, physx_PxBase const* const* observables_pod, uint32_t observableCount) {
        physx::PxPhysics* self_ = reinterpret_cast<physx::PxPhysics*>(self__pod);
        physx::PxDeletionListener& observer = reinterpret_cast<physx::PxDeletionListener&>(*observer_pod);
        physx::PxBase const* const* observables = reinterpret_cast<physx::PxBase const* const*>(observables_pod);
        self_->unregisterDeletionListenerObjects(observer, observables, observableCount);
    }

    physx_PxInsertionCallback* PxPhysics_getPhysicsInsertionCallback(physx_PxPhysics* self__pod) {
        physx::PxPhysics* self_ = reinterpret_cast<physx::PxPhysics*>(self__pod);
        physx::PxInsertionCallback& return_val = self_->getPhysicsInsertionCallback();
        auto return_val_pod = reinterpret_cast<physx_PxInsertionCallback*>(&return_val);
        return return_val_pod;
    }

    physx_PxPhysics* phys_PxCreatePhysics(uint32_t version, physx_PxFoundation* foundation_pod, physx_PxTolerancesScale const* scale_pod, bool trackOutstandingAllocations, physx_PxPvd* pvd_pod, physx_PxOmniPvd* omniPvd_pod) {
        physx::PxFoundation& foundation = reinterpret_cast<physx::PxFoundation&>(*foundation_pod);
        physx::PxTolerancesScale const& scale = reinterpret_cast<physx::PxTolerancesScale const&>(*scale_pod);
        physx::PxPvd* pvd = reinterpret_cast<physx::PxPvd*>(pvd_pod);
        physx::PxOmniPvd* omniPvd = reinterpret_cast<physx::PxOmniPvd*>(omniPvd_pod);
        physx::PxPhysics* return_val = PxCreatePhysics(version, foundation, scale, trackOutstandingAllocations, pvd, omniPvd);
        auto return_val_pod = reinterpret_cast<physx_PxPhysics*>(return_val);
        return return_val_pod;
    }

    physx_PxPhysics* phys_PxGetPhysics() {
        physx::PxPhysics& return_val = PxGetPhysics();
        auto return_val_pod = reinterpret_cast<physx_PxPhysics*>(&return_val);
        return return_val_pod;
    }

    physx_PxActorShape PxActorShape_new() {
        PxActorShape return_val;
        physx_PxActorShape return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxActorShape PxActorShape_new_1(physx_PxRigidActor* a_pod, physx_PxShape* s_pod) {
        physx::PxRigidActor* a = reinterpret_cast<physx::PxRigidActor*>(a_pod);
        physx::PxShape* s = reinterpret_cast<physx::PxShape*>(s_pod);
        PxActorShape return_val(a, s);
        physx_PxActorShape return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxQueryCache PxQueryCache_new() {
        PxQueryCache return_val;
        physx_PxQueryCache return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxQueryCache PxQueryCache_new_1(physx_PxShape* s_pod, uint32_t findex) {
        physx::PxShape* s = reinterpret_cast<physx::PxShape*>(s_pod);
        PxQueryCache return_val(s, findex);
        physx_PxQueryCache return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxQueryFilterData PxQueryFilterData_new() {
        PxQueryFilterData return_val;
        physx_PxQueryFilterData return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxQueryFilterData PxQueryFilterData_new_1(physx_PxFilterData const* fd_pod, PxQueryFlags f_pod) {
        physx::PxFilterData const& fd = reinterpret_cast<physx::PxFilterData const&>(*fd_pod);
        auto f = physx::PxQueryFlags(f_pod);
        PxQueryFilterData return_val(fd, f);
        physx_PxQueryFilterData return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxQueryFilterData PxQueryFilterData_new_2(PxQueryFlags f_pod) {
        auto f = physx::PxQueryFlags(f_pod);
        PxQueryFilterData return_val(f);
        physx_PxQueryFilterData return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    PxQueryHitType PxQueryFilterCallback_preFilter(physx_PxQueryFilterCallback* self__pod, physx_PxFilterData const* filterData_pod, physx_PxShape const* shape_pod, physx_PxRigidActor const* actor_pod, PxHitFlags* queryFlags_pod) {
        physx::PxQueryFilterCallback* self_ = reinterpret_cast<physx::PxQueryFilterCallback*>(self__pod);
        physx::PxFilterData const& filterData = reinterpret_cast<physx::PxFilterData const&>(*filterData_pod);
        physx::PxShape const* shape = reinterpret_cast<physx::PxShape const*>(shape_pod);
        physx::PxRigidActor const* actor = reinterpret_cast<physx::PxRigidActor const*>(actor_pod);
        physx::PxHitFlags& queryFlags = reinterpret_cast<physx::PxHitFlags&>(*queryFlags_pod);
        physx::PxQueryHitType::Enum return_val = self_->preFilter(filterData, shape, actor, queryFlags);
        PxQueryHitType return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    PxQueryHitType PxQueryFilterCallback_postFilter(physx_PxQueryFilterCallback* self__pod, physx_PxFilterData const* filterData_pod, physx_PxQueryHit const* hit_pod, physx_PxShape const* shape_pod, physx_PxRigidActor const* actor_pod) {
        physx::PxQueryFilterCallback* self_ = reinterpret_cast<physx::PxQueryFilterCallback*>(self__pod);
        physx::PxFilterData const& filterData = reinterpret_cast<physx::PxFilterData const&>(*filterData_pod);
        physx::PxQueryHit const& hit = reinterpret_cast<physx::PxQueryHit const&>(*hit_pod);
        physx::PxShape const* shape = reinterpret_cast<physx::PxShape const*>(shape_pod);
        physx::PxRigidActor const* actor = reinterpret_cast<physx::PxRigidActor const*>(actor_pod);
        physx::PxQueryHitType::Enum return_val = self_->postFilter(filterData, hit, shape, actor);
        PxQueryHitType return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxQueryFilterCallback_delete(physx_PxQueryFilterCallback* self__pod) {
        physx::PxQueryFilterCallback* self_ = reinterpret_cast<physx::PxQueryFilterCallback*>(self__pod);
        delete self_;
    }

    void PxRigidDynamic_setKinematicTarget(physx_PxRigidDynamic* self__pod, physx_PxTransform const* destination_pod) {
        physx::PxRigidDynamic* self_ = reinterpret_cast<physx::PxRigidDynamic*>(self__pod);
        physx::PxTransform const& destination = reinterpret_cast<physx::PxTransform const&>(*destination_pod);
        self_->setKinematicTarget(destination);
    }

    bool PxRigidDynamic_getKinematicTarget(physx_PxRigidDynamic const* self__pod, physx_PxTransform* target_pod) {
        physx::PxRigidDynamic const* self_ = reinterpret_cast<physx::PxRigidDynamic const*>(self__pod);
        physx::PxTransform& target = reinterpret_cast<physx::PxTransform&>(*target_pod);
        bool return_val = self_->getKinematicTarget(target);
        return return_val;
    }

    bool PxRigidDynamic_isSleeping(physx_PxRigidDynamic const* self__pod) {
        physx::PxRigidDynamic const* self_ = reinterpret_cast<physx::PxRigidDynamic const*>(self__pod);
        bool return_val = self_->isSleeping();
        return return_val;
    }

    void PxRigidDynamic_setSleepThreshold(physx_PxRigidDynamic* self__pod, float threshold) {
        physx::PxRigidDynamic* self_ = reinterpret_cast<physx::PxRigidDynamic*>(self__pod);
        self_->setSleepThreshold(threshold);
    }

    float PxRigidDynamic_getSleepThreshold(physx_PxRigidDynamic const* self__pod) {
        physx::PxRigidDynamic const* self_ = reinterpret_cast<physx::PxRigidDynamic const*>(self__pod);
        float return_val = self_->getSleepThreshold();
        return return_val;
    }

    void PxRigidDynamic_setStabilizationThreshold(physx_PxRigidDynamic* self__pod, float threshold) {
        physx::PxRigidDynamic* self_ = reinterpret_cast<physx::PxRigidDynamic*>(self__pod);
        self_->setStabilizationThreshold(threshold);
    }

    float PxRigidDynamic_getStabilizationThreshold(physx_PxRigidDynamic const* self__pod) {
        physx::PxRigidDynamic const* self_ = reinterpret_cast<physx::PxRigidDynamic const*>(self__pod);
        float return_val = self_->getStabilizationThreshold();
        return return_val;
    }

    PxRigidDynamicLockFlags PxRigidDynamic_getRigidDynamicLockFlags(physx_PxRigidDynamic const* self__pod) {
        physx::PxRigidDynamic const* self_ = reinterpret_cast<physx::PxRigidDynamic const*>(self__pod);
        physx::PxRigidDynamicLockFlags return_val = self_->getRigidDynamicLockFlags();
        PxRigidDynamicLockFlags return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxRigidDynamic_setRigidDynamicLockFlag(physx_PxRigidDynamic* self__pod, PxRigidDynamicLockFlag flag_pod, bool value) {
        physx::PxRigidDynamic* self_ = reinterpret_cast<physx::PxRigidDynamic*>(self__pod);
        auto flag = static_cast<physx::PxRigidDynamicLockFlag::Enum>(flag_pod);
        self_->setRigidDynamicLockFlag(flag, value);
    }

    void PxRigidDynamic_setRigidDynamicLockFlags(physx_PxRigidDynamic* self__pod, PxRigidDynamicLockFlags flags_pod) {
        physx::PxRigidDynamic* self_ = reinterpret_cast<physx::PxRigidDynamic*>(self__pod);
        auto flags = physx::PxRigidDynamicLockFlags(flags_pod);
        self_->setRigidDynamicLockFlags(flags);
    }

    physx_PxVec3 PxRigidDynamic_getLinearVelocity(physx_PxRigidDynamic const* self__pod) {
        physx::PxRigidDynamic const* self_ = reinterpret_cast<physx::PxRigidDynamic const*>(self__pod);
        physx::PxVec3 return_val = self_->getLinearVelocity();
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxRigidDynamic_setLinearVelocity(physx_PxRigidDynamic* self__pod, physx_PxVec3 const* linVel_pod, bool autowake) {
        physx::PxRigidDynamic* self_ = reinterpret_cast<physx::PxRigidDynamic*>(self__pod);
        physx::PxVec3 const& linVel = reinterpret_cast<physx::PxVec3 const&>(*linVel_pod);
        self_->setLinearVelocity(linVel, autowake);
    }

    physx_PxVec3 PxRigidDynamic_getAngularVelocity(physx_PxRigidDynamic const* self__pod) {
        physx::PxRigidDynamic const* self_ = reinterpret_cast<physx::PxRigidDynamic const*>(self__pod);
        physx::PxVec3 return_val = self_->getAngularVelocity();
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxRigidDynamic_setAngularVelocity(physx_PxRigidDynamic* self__pod, physx_PxVec3 const* angVel_pod, bool autowake) {
        physx::PxRigidDynamic* self_ = reinterpret_cast<physx::PxRigidDynamic*>(self__pod);
        physx::PxVec3 const& angVel = reinterpret_cast<physx::PxVec3 const&>(*angVel_pod);
        self_->setAngularVelocity(angVel, autowake);
    }

    void PxRigidDynamic_setWakeCounter(physx_PxRigidDynamic* self__pod, float wakeCounterValue) {
        physx::PxRigidDynamic* self_ = reinterpret_cast<physx::PxRigidDynamic*>(self__pod);
        self_->setWakeCounter(wakeCounterValue);
    }

    float PxRigidDynamic_getWakeCounter(physx_PxRigidDynamic const* self__pod) {
        physx::PxRigidDynamic const* self_ = reinterpret_cast<physx::PxRigidDynamic const*>(self__pod);
        float return_val = self_->getWakeCounter();
        return return_val;
    }

    void PxRigidDynamic_wakeUp(physx_PxRigidDynamic* self__pod) {
        physx::PxRigidDynamic* self_ = reinterpret_cast<physx::PxRigidDynamic*>(self__pod);
        self_->wakeUp();
    }

    void PxRigidDynamic_putToSleep(physx_PxRigidDynamic* self__pod) {
        physx::PxRigidDynamic* self_ = reinterpret_cast<physx::PxRigidDynamic*>(self__pod);
        self_->putToSleep();
    }

    void PxRigidDynamic_setSolverIterationCounts(physx_PxRigidDynamic* self__pod, uint32_t minPositionIters, uint32_t minVelocityIters) {
        physx::PxRigidDynamic* self_ = reinterpret_cast<physx::PxRigidDynamic*>(self__pod);
        self_->setSolverIterationCounts(minPositionIters, minVelocityIters);
    }

    void PxRigidDynamic_getSolverIterationCounts(physx_PxRigidDynamic const* self__pod, uint32_t* minPositionIters_pod, uint32_t* minVelocityIters_pod) {
        physx::PxRigidDynamic const* self_ = reinterpret_cast<physx::PxRigidDynamic const*>(self__pod);
        uint32_t& minPositionIters = *minPositionIters_pod;
        uint32_t& minVelocityIters = *minVelocityIters_pod;
        self_->getSolverIterationCounts(minPositionIters, minVelocityIters);
    }

    float PxRigidDynamic_getContactReportThreshold(physx_PxRigidDynamic const* self__pod) {
        physx::PxRigidDynamic const* self_ = reinterpret_cast<physx::PxRigidDynamic const*>(self__pod);
        float return_val = self_->getContactReportThreshold();
        return return_val;
    }

    void PxRigidDynamic_setContactReportThreshold(physx_PxRigidDynamic* self__pod, float threshold) {
        physx::PxRigidDynamic* self_ = reinterpret_cast<physx::PxRigidDynamic*>(self__pod);
        self_->setContactReportThreshold(threshold);
    }

    char const* PxRigidDynamic_getConcreteTypeName(physx_PxRigidDynamic const* self__pod) {
        physx::PxRigidDynamic const* self_ = reinterpret_cast<physx::PxRigidDynamic const*>(self__pod);
        char const* return_val = self_->getConcreteTypeName();
        return return_val;
    }

    char const* PxRigidStatic_getConcreteTypeName(physx_PxRigidStatic const* self__pod) {
        physx::PxRigidStatic const* self_ = reinterpret_cast<physx::PxRigidStatic const*>(self__pod);
        char const* return_val = self_->getConcreteTypeName();
        return return_val;
    }

    physx_PxSceneQueryDesc PxSceneQueryDesc_new() {
        PxSceneQueryDesc return_val;
        physx_PxSceneQueryDesc return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxSceneQueryDesc_setToDefault(physx_PxSceneQueryDesc* self__pod) {
        physx::PxSceneQueryDesc* self_ = reinterpret_cast<physx::PxSceneQueryDesc*>(self__pod);
        self_->setToDefault();
    }

    bool PxSceneQueryDesc_isValid(physx_PxSceneQueryDesc const* self__pod) {
        physx::PxSceneQueryDesc const* self_ = reinterpret_cast<physx::PxSceneQueryDesc const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    void PxSceneQuerySystemBase_setDynamicTreeRebuildRateHint(physx_PxSceneQuerySystemBase* self__pod, uint32_t dynamicTreeRebuildRateHint) {
        physx::PxSceneQuerySystemBase* self_ = reinterpret_cast<physx::PxSceneQuerySystemBase*>(self__pod);
        self_->setDynamicTreeRebuildRateHint(dynamicTreeRebuildRateHint);
    }

    uint32_t PxSceneQuerySystemBase_getDynamicTreeRebuildRateHint(physx_PxSceneQuerySystemBase const* self__pod) {
        physx::PxSceneQuerySystemBase const* self_ = reinterpret_cast<physx::PxSceneQuerySystemBase const*>(self__pod);
        uint32_t return_val = self_->getDynamicTreeRebuildRateHint();
        return return_val;
    }

    void PxSceneQuerySystemBase_forceRebuildDynamicTree(physx_PxSceneQuerySystemBase* self__pod, uint32_t prunerIndex) {
        physx::PxSceneQuerySystemBase* self_ = reinterpret_cast<physx::PxSceneQuerySystemBase*>(self__pod);
        self_->forceRebuildDynamicTree(prunerIndex);
    }

    void PxSceneQuerySystemBase_setUpdateMode(physx_PxSceneQuerySystemBase* self__pod, PxSceneQueryUpdateMode updateMode_pod) {
        physx::PxSceneQuerySystemBase* self_ = reinterpret_cast<physx::PxSceneQuerySystemBase*>(self__pod);
        auto updateMode = static_cast<physx::PxSceneQueryUpdateMode::Enum>(updateMode_pod);
        self_->setUpdateMode(updateMode);
    }

    PxSceneQueryUpdateMode PxSceneQuerySystemBase_getUpdateMode(physx_PxSceneQuerySystemBase const* self__pod) {
        physx::PxSceneQuerySystemBase const* self_ = reinterpret_cast<physx::PxSceneQuerySystemBase const*>(self__pod);
        physx::PxSceneQueryUpdateMode::Enum return_val = self_->getUpdateMode();
        PxSceneQueryUpdateMode return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    uint32_t PxSceneQuerySystemBase_getStaticTimestamp(physx_PxSceneQuerySystemBase const* self__pod) {
        physx::PxSceneQuerySystemBase const* self_ = reinterpret_cast<physx::PxSceneQuerySystemBase const*>(self__pod);
        uint32_t return_val = self_->getStaticTimestamp();
        return return_val;
    }

    void PxSceneQuerySystemBase_flushUpdates(physx_PxSceneQuerySystemBase* self__pod) {
        physx::PxSceneQuerySystemBase* self_ = reinterpret_cast<physx::PxSceneQuerySystemBase*>(self__pod);
        self_->flushUpdates();
    }

    bool PxSceneQuerySystemBase_raycast(physx_PxSceneQuerySystemBase const* self__pod, physx_PxVec3 const* origin_pod, physx_PxVec3 const* unitDir_pod, float distance, physx_PxRaycastCallback* hitCall_pod, PxHitFlags hitFlags_pod, physx_PxQueryFilterData const* filterData_pod, physx_PxQueryFilterCallback* filterCall_pod, physx_PxQueryCache const* cache_pod, PxGeometryQueryFlags queryFlags_pod) {
        physx::PxSceneQuerySystemBase const* self_ = reinterpret_cast<physx::PxSceneQuerySystemBase const*>(self__pod);
        physx::PxVec3 const& origin = reinterpret_cast<physx::PxVec3 const&>(*origin_pod);
        physx::PxVec3 const& unitDir = reinterpret_cast<physx::PxVec3 const&>(*unitDir_pod);
        physx::PxRaycastCallback& hitCall = reinterpret_cast<physx::PxRaycastCallback&>(*hitCall_pod);
        auto hitFlags = physx::PxHitFlags(hitFlags_pod);
        physx::PxQueryFilterData const& filterData = reinterpret_cast<physx::PxQueryFilterData const&>(*filterData_pod);
        physx::PxQueryFilterCallback* filterCall = reinterpret_cast<physx::PxQueryFilterCallback*>(filterCall_pod);
        physx::PxQueryCache const* cache = reinterpret_cast<physx::PxQueryCache const*>(cache_pod);
        auto queryFlags = physx::PxGeometryQueryFlags(queryFlags_pod);
        bool return_val = self_->raycast(origin, unitDir, distance, hitCall, hitFlags, filterData, filterCall, cache, queryFlags);
        return return_val;
    }

    bool PxSceneQuerySystemBase_sweep(physx_PxSceneQuerySystemBase const* self__pod, physx_PxGeometry const* geometry_pod, physx_PxTransform const* pose_pod, physx_PxVec3 const* unitDir_pod, float distance, physx_PxSweepCallback* hitCall_pod, PxHitFlags hitFlags_pod, physx_PxQueryFilterData const* filterData_pod, physx_PxQueryFilterCallback* filterCall_pod, physx_PxQueryCache const* cache_pod, float inflation, PxGeometryQueryFlags queryFlags_pod) {
        physx::PxSceneQuerySystemBase const* self_ = reinterpret_cast<physx::PxSceneQuerySystemBase const*>(self__pod);
        physx::PxGeometry const& geometry = reinterpret_cast<physx::PxGeometry const&>(*geometry_pod);
        physx::PxTransform const& pose = reinterpret_cast<physx::PxTransform const&>(*pose_pod);
        physx::PxVec3 const& unitDir = reinterpret_cast<physx::PxVec3 const&>(*unitDir_pod);
        physx::PxSweepCallback& hitCall = reinterpret_cast<physx::PxSweepCallback&>(*hitCall_pod);
        auto hitFlags = physx::PxHitFlags(hitFlags_pod);
        physx::PxQueryFilterData const& filterData = reinterpret_cast<physx::PxQueryFilterData const&>(*filterData_pod);
        physx::PxQueryFilterCallback* filterCall = reinterpret_cast<physx::PxQueryFilterCallback*>(filterCall_pod);
        physx::PxQueryCache const* cache = reinterpret_cast<physx::PxQueryCache const*>(cache_pod);
        auto queryFlags = physx::PxGeometryQueryFlags(queryFlags_pod);
        bool return_val = self_->sweep(geometry, pose, unitDir, distance, hitCall, hitFlags, filterData, filterCall, cache, inflation, queryFlags);
        return return_val;
    }

    bool PxSceneQuerySystemBase_overlap(physx_PxSceneQuerySystemBase const* self__pod, physx_PxGeometry const* geometry_pod, physx_PxTransform const* pose_pod, physx_PxOverlapCallback* hitCall_pod, physx_PxQueryFilterData const* filterData_pod, physx_PxQueryFilterCallback* filterCall_pod, physx_PxQueryCache const* cache_pod, PxGeometryQueryFlags queryFlags_pod) {
        physx::PxSceneQuerySystemBase const* self_ = reinterpret_cast<physx::PxSceneQuerySystemBase const*>(self__pod);
        physx::PxGeometry const& geometry = reinterpret_cast<physx::PxGeometry const&>(*geometry_pod);
        physx::PxTransform const& pose = reinterpret_cast<physx::PxTransform const&>(*pose_pod);
        physx::PxOverlapCallback& hitCall = reinterpret_cast<physx::PxOverlapCallback&>(*hitCall_pod);
        physx::PxQueryFilterData const& filterData = reinterpret_cast<physx::PxQueryFilterData const&>(*filterData_pod);
        physx::PxQueryFilterCallback* filterCall = reinterpret_cast<physx::PxQueryFilterCallback*>(filterCall_pod);
        physx::PxQueryCache const* cache = reinterpret_cast<physx::PxQueryCache const*>(cache_pod);
        auto queryFlags = physx::PxGeometryQueryFlags(queryFlags_pod);
        bool return_val = self_->overlap(geometry, pose, hitCall, filterData, filterCall, cache, queryFlags);
        return return_val;
    }

    void PxSceneSQSystem_setSceneQueryUpdateMode(physx_PxSceneSQSystem* self__pod, PxSceneQueryUpdateMode updateMode_pod) {
        physx::PxSceneSQSystem* self_ = reinterpret_cast<physx::PxSceneSQSystem*>(self__pod);
        auto updateMode = static_cast<physx::PxSceneQueryUpdateMode::Enum>(updateMode_pod);
        self_->setSceneQueryUpdateMode(updateMode);
    }

    PxSceneQueryUpdateMode PxSceneSQSystem_getSceneQueryUpdateMode(physx_PxSceneSQSystem const* self__pod) {
        physx::PxSceneSQSystem const* self_ = reinterpret_cast<physx::PxSceneSQSystem const*>(self__pod);
        physx::PxSceneQueryUpdateMode::Enum return_val = self_->getSceneQueryUpdateMode();
        PxSceneQueryUpdateMode return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    uint32_t PxSceneSQSystem_getSceneQueryStaticTimestamp(physx_PxSceneSQSystem const* self__pod) {
        physx::PxSceneSQSystem const* self_ = reinterpret_cast<physx::PxSceneSQSystem const*>(self__pod);
        uint32_t return_val = self_->getSceneQueryStaticTimestamp();
        return return_val;
    }

    void PxSceneSQSystem_flushQueryUpdates(physx_PxSceneSQSystem* self__pod) {
        physx::PxSceneSQSystem* self_ = reinterpret_cast<physx::PxSceneSQSystem*>(self__pod);
        self_->flushQueryUpdates();
    }

    void PxSceneSQSystem_forceDynamicTreeRebuild(physx_PxSceneSQSystem* self__pod, bool rebuildStaticStructure, bool rebuildDynamicStructure) {
        physx::PxSceneSQSystem* self_ = reinterpret_cast<physx::PxSceneSQSystem*>(self__pod);
        self_->forceDynamicTreeRebuild(rebuildStaticStructure, rebuildDynamicStructure);
    }

    PxPruningStructureType PxSceneSQSystem_getStaticStructure(physx_PxSceneSQSystem const* self__pod) {
        physx::PxSceneSQSystem const* self_ = reinterpret_cast<physx::PxSceneSQSystem const*>(self__pod);
        physx::PxPruningStructureType::Enum return_val = self_->getStaticStructure();
        PxPruningStructureType return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    PxPruningStructureType PxSceneSQSystem_getDynamicStructure(physx_PxSceneSQSystem const* self__pod) {
        physx::PxSceneSQSystem const* self_ = reinterpret_cast<physx::PxSceneSQSystem const*>(self__pod);
        physx::PxPruningStructureType::Enum return_val = self_->getDynamicStructure();
        PxPruningStructureType return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxSceneSQSystem_sceneQueriesUpdate(physx_PxSceneSQSystem* self__pod, physx_PxBaseTask* completionTask_pod, bool controlSimulation) {
        physx::PxSceneSQSystem* self_ = reinterpret_cast<physx::PxSceneSQSystem*>(self__pod);
        physx::PxBaseTask* completionTask = reinterpret_cast<physx::PxBaseTask*>(completionTask_pod);
        self_->sceneQueriesUpdate(completionTask, controlSimulation);
    }

    bool PxSceneSQSystem_checkQueries(physx_PxSceneSQSystem* self__pod, bool block) {
        physx::PxSceneSQSystem* self_ = reinterpret_cast<physx::PxSceneSQSystem*>(self__pod);
        bool return_val = self_->checkQueries(block);
        return return_val;
    }

    bool PxSceneSQSystem_fetchQueries(physx_PxSceneSQSystem* self__pod, bool block) {
        physx::PxSceneSQSystem* self_ = reinterpret_cast<physx::PxSceneSQSystem*>(self__pod);
        bool return_val = self_->fetchQueries(block);
        return return_val;
    }

    void PxSceneQuerySystem_release(physx_PxSceneQuerySystem* self__pod) {
        physx::PxSceneQuerySystem* self_ = reinterpret_cast<physx::PxSceneQuerySystem*>(self__pod);
        self_->release();
    }

    void PxSceneQuerySystem_acquireReference(physx_PxSceneQuerySystem* self__pod) {
        physx::PxSceneQuerySystem* self_ = reinterpret_cast<physx::PxSceneQuerySystem*>(self__pod);
        self_->acquireReference();
    }

    void PxSceneQuerySystem_preallocate(physx_PxSceneQuerySystem* self__pod, uint32_t prunerIndex, uint32_t nbShapes) {
        physx::PxSceneQuerySystem* self_ = reinterpret_cast<physx::PxSceneQuerySystem*>(self__pod);
        self_->preallocate(prunerIndex, nbShapes);
    }

    void PxSceneQuerySystem_flushMemory(physx_PxSceneQuerySystem* self__pod) {
        physx::PxSceneQuerySystem* self_ = reinterpret_cast<physx::PxSceneQuerySystem*>(self__pod);
        self_->flushMemory();
    }

    void PxSceneQuerySystem_addSQShape(physx_PxSceneQuerySystem* self__pod, physx_PxRigidActor const* actor_pod, physx_PxShape const* shape_pod, physx_PxBounds3 const* bounds_pod, physx_PxTransform const* transform_pod, uint32_t const* compoundHandle, bool hasPruningStructure) {
        physx::PxSceneQuerySystem* self_ = reinterpret_cast<physx::PxSceneQuerySystem*>(self__pod);
        physx::PxRigidActor const& actor = reinterpret_cast<physx::PxRigidActor const&>(*actor_pod);
        physx::PxShape const& shape = reinterpret_cast<physx::PxShape const&>(*shape_pod);
        physx::PxBounds3 const& bounds = reinterpret_cast<physx::PxBounds3 const&>(*bounds_pod);
        physx::PxTransform const& transform = reinterpret_cast<physx::PxTransform const&>(*transform_pod);
        self_->addSQShape(actor, shape, bounds, transform, compoundHandle, hasPruningStructure);
    }

    void PxSceneQuerySystem_removeSQShape(physx_PxSceneQuerySystem* self__pod, physx_PxRigidActor const* actor_pod, physx_PxShape const* shape_pod) {
        physx::PxSceneQuerySystem* self_ = reinterpret_cast<physx::PxSceneQuerySystem*>(self__pod);
        physx::PxRigidActor const& actor = reinterpret_cast<physx::PxRigidActor const&>(*actor_pod);
        physx::PxShape const& shape = reinterpret_cast<physx::PxShape const&>(*shape_pod);
        self_->removeSQShape(actor, shape);
    }

    void PxSceneQuerySystem_updateSQShape(physx_PxSceneQuerySystem* self__pod, physx_PxRigidActor const* actor_pod, physx_PxShape const* shape_pod, physx_PxTransform const* transform_pod) {
        physx::PxSceneQuerySystem* self_ = reinterpret_cast<physx::PxSceneQuerySystem*>(self__pod);
        physx::PxRigidActor const& actor = reinterpret_cast<physx::PxRigidActor const&>(*actor_pod);
        physx::PxShape const& shape = reinterpret_cast<physx::PxShape const&>(*shape_pod);
        physx::PxTransform const& transform = reinterpret_cast<physx::PxTransform const&>(*transform_pod);
        self_->updateSQShape(actor, shape, transform);
    }

    uint32_t PxSceneQuerySystem_addSQCompound(physx_PxSceneQuerySystem* self__pod, physx_PxRigidActor const* actor_pod, physx_PxShape const** shapes_pod, physx_PxBVH const* bvh_pod, physx_PxTransform const* transforms_pod) {
        physx::PxSceneQuerySystem* self_ = reinterpret_cast<physx::PxSceneQuerySystem*>(self__pod);
        physx::PxRigidActor const& actor = reinterpret_cast<physx::PxRigidActor const&>(*actor_pod);
        physx::PxShape const** shapes = reinterpret_cast<physx::PxShape const**>(shapes_pod);
        physx::PxBVH const& bvh = reinterpret_cast<physx::PxBVH const&>(*bvh_pod);
        physx::PxTransform const* transforms = reinterpret_cast<physx::PxTransform const*>(transforms_pod);
        uint32_t return_val = self_->addSQCompound(actor, shapes, bvh, transforms);
        return return_val;
    }

    void PxSceneQuerySystem_removeSQCompound(physx_PxSceneQuerySystem* self__pod, uint32_t compoundHandle) {
        physx::PxSceneQuerySystem* self_ = reinterpret_cast<physx::PxSceneQuerySystem*>(self__pod);
        self_->removeSQCompound(compoundHandle);
    }

    void PxSceneQuerySystem_updateSQCompound(physx_PxSceneQuerySystem* self__pod, uint32_t compoundHandle, physx_PxTransform const* compoundTransform_pod) {
        physx::PxSceneQuerySystem* self_ = reinterpret_cast<physx::PxSceneQuerySystem*>(self__pod);
        physx::PxTransform const& compoundTransform = reinterpret_cast<physx::PxTransform const&>(*compoundTransform_pod);
        self_->updateSQCompound(compoundHandle, compoundTransform);
    }

    void PxSceneQuerySystem_shiftOrigin(physx_PxSceneQuerySystem* self__pod, physx_PxVec3 const* shift_pod) {
        physx::PxSceneQuerySystem* self_ = reinterpret_cast<physx::PxSceneQuerySystem*>(self__pod);
        physx::PxVec3 const& shift = reinterpret_cast<physx::PxVec3 const&>(*shift_pod);
        self_->shiftOrigin(shift);
    }

    void PxSceneQuerySystem_merge(physx_PxSceneQuerySystem* self__pod, physx_PxPruningStructure const* pruningStructure_pod) {
        physx::PxSceneQuerySystem* self_ = reinterpret_cast<physx::PxSceneQuerySystem*>(self__pod);
        physx::PxPruningStructure const& pruningStructure = reinterpret_cast<physx::PxPruningStructure const&>(*pruningStructure_pod);
        self_->merge(pruningStructure);
    }

    uint32_t PxSceneQuerySystem_getHandle(physx_PxSceneQuerySystem const* self__pod, physx_PxRigidActor const* actor_pod, physx_PxShape const* shape_pod, uint32_t* prunerIndex_pod) {
        physx::PxSceneQuerySystem const* self_ = reinterpret_cast<physx::PxSceneQuerySystem const*>(self__pod);
        physx::PxRigidActor const& actor = reinterpret_cast<physx::PxRigidActor const&>(*actor_pod);
        physx::PxShape const& shape = reinterpret_cast<physx::PxShape const&>(*shape_pod);
        uint32_t& prunerIndex = *prunerIndex_pod;
        uint32_t return_val = self_->getHandle(actor, shape, prunerIndex);
        return return_val;
    }

    void PxSceneQuerySystem_sync(physx_PxSceneQuerySystem* self__pod, uint32_t prunerIndex, uint32_t const* handles, uint32_t const* indices, physx_PxBounds3 const* bounds_pod, physx_PxTransformPadded const* transforms_pod, uint32_t count, physx_PxBitMap const* ignoredIndices_pod) {
        physx::PxSceneQuerySystem* self_ = reinterpret_cast<physx::PxSceneQuerySystem*>(self__pod);
        physx::PxBounds3 const* bounds = reinterpret_cast<physx::PxBounds3 const*>(bounds_pod);
        physx::PxTransformPadded const* transforms = reinterpret_cast<physx::PxTransformPadded const*>(transforms_pod);
        physx::PxBitMap const& ignoredIndices = reinterpret_cast<physx::PxBitMap const&>(*ignoredIndices_pod);
        self_->sync(prunerIndex, handles, indices, bounds, transforms, count, ignoredIndices);
    }

    void PxSceneQuerySystem_finalizeUpdates(physx_PxSceneQuerySystem* self__pod) {
        physx::PxSceneQuerySystem* self_ = reinterpret_cast<physx::PxSceneQuerySystem*>(self__pod);
        self_->finalizeUpdates();
    }

    void* PxSceneQuerySystem_prepareSceneQueryBuildStep(physx_PxSceneQuerySystem* self__pod, uint32_t prunerIndex) {
        physx::PxSceneQuerySystem* self_ = reinterpret_cast<physx::PxSceneQuerySystem*>(self__pod);
        void* return_val = self_->prepareSceneQueryBuildStep(prunerIndex);
        return return_val;
    }

    void PxSceneQuerySystem_sceneQueryBuildStep(physx_PxSceneQuerySystem* self__pod, void* handle) {
        physx::PxSceneQuerySystem* self_ = reinterpret_cast<physx::PxSceneQuerySystem*>(self__pod);
        self_->sceneQueryBuildStep(handle);
    }

    physx_PxBroadPhaseDesc PxBroadPhaseDesc_new(PxBroadPhaseType type_pod) {
        auto type = static_cast<physx::PxBroadPhaseType::Enum>(type_pod);
        PxBroadPhaseDesc return_val(type);
        physx_PxBroadPhaseDesc return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxBroadPhaseDesc_isValid(physx_PxBroadPhaseDesc const* self__pod) {
        physx::PxBroadPhaseDesc const* self_ = reinterpret_cast<physx::PxBroadPhaseDesc const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    uint32_t phys_PxGetBroadPhaseStaticFilterGroup() {
        uint32_t return_val = PxGetBroadPhaseStaticFilterGroup();
        return return_val;
    }

    uint32_t phys_PxGetBroadPhaseDynamicFilterGroup(uint32_t id) {
        uint32_t return_val = PxGetBroadPhaseDynamicFilterGroup(id);
        return return_val;
    }

    uint32_t phys_PxGetBroadPhaseKinematicFilterGroup(uint32_t id) {
        uint32_t return_val = PxGetBroadPhaseKinematicFilterGroup(id);
        return return_val;
    }

    physx_PxBroadPhaseUpdateData PxBroadPhaseUpdateData_new(uint32_t const* created, uint32_t nbCreated, uint32_t const* updated, uint32_t nbUpdated, uint32_t const* removed, uint32_t nbRemoved, physx_PxBounds3 const* bounds_pod, uint32_t const* groups, float const* distances, uint32_t capacity) {
        physx::PxBounds3 const* bounds = reinterpret_cast<physx::PxBounds3 const*>(bounds_pod);
        PxBroadPhaseUpdateData return_val(created, nbCreated, updated, nbUpdated, removed, nbRemoved, bounds, groups, distances, capacity);
        physx_PxBroadPhaseUpdateData return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxBroadPhaseResults PxBroadPhaseResults_new() {
        PxBroadPhaseResults return_val;
        physx_PxBroadPhaseResults return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    uint32_t PxBroadPhaseRegions_getNbRegions(physx_PxBroadPhaseRegions const* self__pod) {
        physx::PxBroadPhaseRegions const* self_ = reinterpret_cast<physx::PxBroadPhaseRegions const*>(self__pod);
        uint32_t return_val = self_->getNbRegions();
        return return_val;
    }

    uint32_t PxBroadPhaseRegions_getRegions(physx_PxBroadPhaseRegions const* self__pod, physx_PxBroadPhaseRegionInfo* userBuffer_pod, uint32_t bufferSize, uint32_t startIndex) {
        physx::PxBroadPhaseRegions const* self_ = reinterpret_cast<physx::PxBroadPhaseRegions const*>(self__pod);
        physx::PxBroadPhaseRegionInfo* userBuffer = reinterpret_cast<physx::PxBroadPhaseRegionInfo*>(userBuffer_pod);
        uint32_t return_val = self_->getRegions(userBuffer, bufferSize, startIndex);
        return return_val;
    }

    uint32_t PxBroadPhaseRegions_addRegion(physx_PxBroadPhaseRegions* self__pod, physx_PxBroadPhaseRegion const* region_pod, bool populateRegion, physx_PxBounds3 const* bounds_pod, float const* distances) {
        physx::PxBroadPhaseRegions* self_ = reinterpret_cast<physx::PxBroadPhaseRegions*>(self__pod);
        physx::PxBroadPhaseRegion const& region = reinterpret_cast<physx::PxBroadPhaseRegion const&>(*region_pod);
        physx::PxBounds3 const* bounds = reinterpret_cast<physx::PxBounds3 const*>(bounds_pod);
        uint32_t return_val = self_->addRegion(region, populateRegion, bounds, distances);
        return return_val;
    }

    bool PxBroadPhaseRegions_removeRegion(physx_PxBroadPhaseRegions* self__pod, uint32_t handle) {
        physx::PxBroadPhaseRegions* self_ = reinterpret_cast<physx::PxBroadPhaseRegions*>(self__pod);
        bool return_val = self_->removeRegion(handle);
        return return_val;
    }

    uint32_t PxBroadPhaseRegions_getNbOutOfBoundsObjects(physx_PxBroadPhaseRegions const* self__pod) {
        physx::PxBroadPhaseRegions const* self_ = reinterpret_cast<physx::PxBroadPhaseRegions const*>(self__pod);
        uint32_t return_val = self_->getNbOutOfBoundsObjects();
        return return_val;
    }

    uint32_t const* PxBroadPhaseRegions_getOutOfBoundsObjects(physx_PxBroadPhaseRegions const* self__pod) {
        physx::PxBroadPhaseRegions const* self_ = reinterpret_cast<physx::PxBroadPhaseRegions const*>(self__pod);
        uint32_t const* return_val = self_->getOutOfBoundsObjects();
        return return_val;
    }

    void PxBroadPhase_release(physx_PxBroadPhase* self__pod) {
        physx::PxBroadPhase* self_ = reinterpret_cast<physx::PxBroadPhase*>(self__pod);
        self_->release();
    }

    PxBroadPhaseType PxBroadPhase_getType(physx_PxBroadPhase const* self__pod) {
        physx::PxBroadPhase const* self_ = reinterpret_cast<physx::PxBroadPhase const*>(self__pod);
        physx::PxBroadPhaseType::Enum return_val = self_->getType();
        PxBroadPhaseType return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxBroadPhase_getCaps(physx_PxBroadPhase const* self__pod, physx_PxBroadPhaseCaps* caps_pod) {
        physx::PxBroadPhase const* self_ = reinterpret_cast<physx::PxBroadPhase const*>(self__pod);
        physx::PxBroadPhaseCaps& caps = reinterpret_cast<physx::PxBroadPhaseCaps&>(*caps_pod);
        self_->getCaps(caps);
    }

    physx_PxBroadPhaseRegions* PxBroadPhase_getRegions(physx_PxBroadPhase* self__pod) {
        physx::PxBroadPhase* self_ = reinterpret_cast<physx::PxBroadPhase*>(self__pod);
        physx::PxBroadPhaseRegions* return_val = self_->getRegions();
        auto return_val_pod = reinterpret_cast<physx_PxBroadPhaseRegions*>(return_val);
        return return_val_pod;
    }

    physx_PxAllocatorCallback* PxBroadPhase_getAllocator(physx_PxBroadPhase* self__pod) {
        physx::PxBroadPhase* self_ = reinterpret_cast<physx::PxBroadPhase*>(self__pod);
        physx::PxAllocatorCallback* return_val = self_->getAllocator();
        auto return_val_pod = reinterpret_cast<physx_PxAllocatorCallback*>(return_val);
        return return_val_pod;
    }

    uint64_t PxBroadPhase_getContextID(physx_PxBroadPhase const* self__pod) {
        physx::PxBroadPhase const* self_ = reinterpret_cast<physx::PxBroadPhase const*>(self__pod);
        uint64_t return_val = self_->getContextID();
        return return_val;
    }

    void PxBroadPhase_setScratchBlock(physx_PxBroadPhase* self__pod, void* scratchBlock, uint32_t size) {
        physx::PxBroadPhase* self_ = reinterpret_cast<physx::PxBroadPhase*>(self__pod);
        self_->setScratchBlock(scratchBlock, size);
    }

    void PxBroadPhase_update(physx_PxBroadPhase* self__pod, physx_PxBroadPhaseUpdateData const* updateData_pod, physx_PxBaseTask* continuation_pod) {
        physx::PxBroadPhase* self_ = reinterpret_cast<physx::PxBroadPhase*>(self__pod);
        physx::PxBroadPhaseUpdateData const& updateData = reinterpret_cast<physx::PxBroadPhaseUpdateData const&>(*updateData_pod);
        physx::PxBaseTask* continuation = reinterpret_cast<physx::PxBaseTask*>(continuation_pod);
        self_->update(updateData, continuation);
    }

    void PxBroadPhase_fetchResults(physx_PxBroadPhase* self__pod, physx_PxBroadPhaseResults* results_pod) {
        physx::PxBroadPhase* self_ = reinterpret_cast<physx::PxBroadPhase*>(self__pod);
        physx::PxBroadPhaseResults& results = reinterpret_cast<physx::PxBroadPhaseResults&>(*results_pod);
        self_->fetchResults(results);
    }

    void PxBroadPhase_update_1(physx_PxBroadPhase* self__pod, physx_PxBroadPhaseResults* results_pod, physx_PxBroadPhaseUpdateData const* updateData_pod) {
        physx::PxBroadPhase* self_ = reinterpret_cast<physx::PxBroadPhase*>(self__pod);
        physx::PxBroadPhaseResults& results = reinterpret_cast<physx::PxBroadPhaseResults&>(*results_pod);
        physx::PxBroadPhaseUpdateData const& updateData = reinterpret_cast<physx::PxBroadPhaseUpdateData const&>(*updateData_pod);
        self_->update(results, updateData);
    }

    physx_PxBroadPhase* phys_PxCreateBroadPhase(physx_PxBroadPhaseDesc const* desc_pod) {
        physx::PxBroadPhaseDesc const& desc = reinterpret_cast<physx::PxBroadPhaseDesc const&>(*desc_pod);
        physx::PxBroadPhase* return_val = PxCreateBroadPhase(desc);
        auto return_val_pod = reinterpret_cast<physx_PxBroadPhase*>(return_val);
        return return_val_pod;
    }

    void PxAABBManager_release(physx_PxAABBManager* self__pod) {
        physx::PxAABBManager* self_ = reinterpret_cast<physx::PxAABBManager*>(self__pod);
        self_->release();
    }

    physx_PxBroadPhase* PxAABBManager_getBroadPhase(physx_PxAABBManager* self__pod) {
        physx::PxAABBManager* self_ = reinterpret_cast<physx::PxAABBManager*>(self__pod);
        physx::PxBroadPhase& return_val = self_->getBroadPhase();
        auto return_val_pod = reinterpret_cast<physx_PxBroadPhase*>(&return_val);
        return return_val_pod;
    }

    physx_PxBounds3 const* PxAABBManager_getBounds(physx_PxAABBManager const* self__pod) {
        physx::PxAABBManager const* self_ = reinterpret_cast<physx::PxAABBManager const*>(self__pod);
        physx::PxBounds3 const* return_val = self_->getBounds();
        auto return_val_pod = reinterpret_cast<physx_PxBounds3 const*>(return_val);
        return return_val_pod;
    }

    float const* PxAABBManager_getDistances(physx_PxAABBManager const* self__pod) {
        physx::PxAABBManager const* self_ = reinterpret_cast<physx::PxAABBManager const*>(self__pod);
        float const* return_val = self_->getDistances();
        return return_val;
    }

    uint32_t const* PxAABBManager_getGroups(physx_PxAABBManager const* self__pod) {
        physx::PxAABBManager const* self_ = reinterpret_cast<physx::PxAABBManager const*>(self__pod);
        uint32_t const* return_val = self_->getGroups();
        return return_val;
    }

    uint32_t PxAABBManager_getCapacity(physx_PxAABBManager const* self__pod) {
        physx::PxAABBManager const* self_ = reinterpret_cast<physx::PxAABBManager const*>(self__pod);
        uint32_t return_val = self_->getCapacity();
        return return_val;
    }

    void PxAABBManager_addObject(physx_PxAABBManager* self__pod, uint32_t index, physx_PxBounds3 const* bounds_pod, uint32_t group, float distance) {
        physx::PxAABBManager* self_ = reinterpret_cast<physx::PxAABBManager*>(self__pod);
        physx::PxBounds3 const& bounds = reinterpret_cast<physx::PxBounds3 const&>(*bounds_pod);
        self_->addObject(index, bounds, group, distance);
    }

    void PxAABBManager_removeObject(physx_PxAABBManager* self__pod, uint32_t index) {
        physx::PxAABBManager* self_ = reinterpret_cast<physx::PxAABBManager*>(self__pod);
        self_->removeObject(index);
    }

    void PxAABBManager_updateObject(physx_PxAABBManager* self__pod, uint32_t index, physx_PxBounds3 const* bounds_pod, float const* distance) {
        physx::PxAABBManager* self_ = reinterpret_cast<physx::PxAABBManager*>(self__pod);
        physx::PxBounds3 const* bounds = reinterpret_cast<physx::PxBounds3 const*>(bounds_pod);
        self_->updateObject(index, bounds, distance);
    }

    void PxAABBManager_update(physx_PxAABBManager* self__pod, physx_PxBaseTask* continuation_pod) {
        physx::PxAABBManager* self_ = reinterpret_cast<physx::PxAABBManager*>(self__pod);
        physx::PxBaseTask* continuation = reinterpret_cast<physx::PxBaseTask*>(continuation_pod);
        self_->update(continuation);
    }

    void PxAABBManager_fetchResults(physx_PxAABBManager* self__pod, physx_PxBroadPhaseResults* results_pod) {
        physx::PxAABBManager* self_ = reinterpret_cast<physx::PxAABBManager*>(self__pod);
        physx::PxBroadPhaseResults& results = reinterpret_cast<physx::PxBroadPhaseResults&>(*results_pod);
        self_->fetchResults(results);
    }

    void PxAABBManager_update_1(physx_PxAABBManager* self__pod, physx_PxBroadPhaseResults* results_pod) {
        physx::PxAABBManager* self_ = reinterpret_cast<physx::PxAABBManager*>(self__pod);
        physx::PxBroadPhaseResults& results = reinterpret_cast<physx::PxBroadPhaseResults&>(*results_pod);
        self_->update(results);
    }

    physx_PxAABBManager* phys_PxCreateAABBManager(physx_PxBroadPhase* broadphase_pod) {
        physx::PxBroadPhase& broadphase = reinterpret_cast<physx::PxBroadPhase&>(*broadphase_pod);
        physx::PxAABBManager* return_val = PxCreateAABBManager(broadphase);
        auto return_val_pod = reinterpret_cast<physx_PxAABBManager*>(return_val);
        return return_val_pod;
    }

    physx_PxSceneLimits PxSceneLimits_new() {
        PxSceneLimits return_val;
        physx_PxSceneLimits return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxSceneLimits_setToDefault(physx_PxSceneLimits* self__pod) {
        physx::PxSceneLimits* self_ = reinterpret_cast<physx::PxSceneLimits*>(self__pod);
        self_->setToDefault();
    }

    bool PxSceneLimits_isValid(physx_PxSceneLimits const* self__pod) {
        physx::PxSceneLimits const* self_ = reinterpret_cast<physx::PxSceneLimits const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    physx_PxgDynamicsMemoryConfig PxgDynamicsMemoryConfig_new() {
        PxgDynamicsMemoryConfig return_val;
        physx_PxgDynamicsMemoryConfig return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxgDynamicsMemoryConfig_isValid(physx_PxgDynamicsMemoryConfig const* self__pod) {
        physx::PxgDynamicsMemoryConfig const* self_ = reinterpret_cast<physx::PxgDynamicsMemoryConfig const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    physx_PxSceneDesc PxSceneDesc_new(physx_PxTolerancesScale const* scale_pod) {
        physx::PxTolerancesScale const& scale = reinterpret_cast<physx::PxTolerancesScale const&>(*scale_pod);
        PxSceneDesc return_val(scale);
        physx_PxSceneDesc return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxSceneDesc_setToDefault(physx_PxSceneDesc* self__pod, physx_PxTolerancesScale const* scale_pod) {
        physx::PxSceneDesc* self_ = reinterpret_cast<physx::PxSceneDesc*>(self__pod);
        physx::PxTolerancesScale const& scale = reinterpret_cast<physx::PxTolerancesScale const&>(*scale_pod);
        self_->setToDefault(scale);
    }

    bool PxSceneDesc_isValid(physx_PxSceneDesc const* self__pod) {
        physx::PxSceneDesc const* self_ = reinterpret_cast<physx::PxSceneDesc const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    physx_PxTolerancesScale const* PxSceneDesc_getTolerancesScale(physx_PxSceneDesc const* self__pod) {
        physx::PxSceneDesc const* self_ = reinterpret_cast<physx::PxSceneDesc const*>(self__pod);
        physx::PxTolerancesScale const& return_val = self_->getTolerancesScale();
        auto return_val_pod = reinterpret_cast<physx_PxTolerancesScale const*>(&return_val);
        return return_val_pod;
    }

    uint32_t PxSimulationStatistics_getNbBroadPhaseAdds(physx_PxSimulationStatistics const* self__pod) {
        physx::PxSimulationStatistics const* self_ = reinterpret_cast<physx::PxSimulationStatistics const*>(self__pod);
        uint32_t return_val = self_->getNbBroadPhaseAdds();
        return return_val;
    }

    uint32_t PxSimulationStatistics_getNbBroadPhaseRemoves(physx_PxSimulationStatistics const* self__pod) {
        physx::PxSimulationStatistics const* self_ = reinterpret_cast<physx::PxSimulationStatistics const*>(self__pod);
        uint32_t return_val = self_->getNbBroadPhaseRemoves();
        return return_val;
    }

    uint32_t PxSimulationStatistics_getRbPairStats(physx_PxSimulationStatistics const* self__pod, RbPairStatsType pairType_pod, PxGeometryType g0_pod, PxGeometryType g1_pod) {
        physx::PxSimulationStatistics const* self_ = reinterpret_cast<physx::PxSimulationStatistics const*>(self__pod);
        auto pairType = static_cast<physx::PxSimulationStatistics::RbPairStatsType>(pairType_pod);
        auto g0 = static_cast<physx::PxGeometryType::Enum>(g0_pod);
        auto g1 = static_cast<physx::PxGeometryType::Enum>(g1_pod);
        uint32_t return_val = self_->getRbPairStats(pairType, g0, g1);
        return return_val;
    }

    physx_PxSimulationStatistics PxSimulationStatistics_new() {
        PxSimulationStatistics return_val;
        physx_PxSimulationStatistics return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxPvdSceneClient_setScenePvdFlag(physx_PxPvdSceneClient* self__pod, PxPvdSceneFlag flag_pod, bool value) {
        physx::PxPvdSceneClient* self_ = reinterpret_cast<physx::PxPvdSceneClient*>(self__pod);
        auto flag = static_cast<physx::PxPvdSceneFlag::Enum>(flag_pod);
        self_->setScenePvdFlag(flag, value);
    }

    void PxPvdSceneClient_setScenePvdFlags(physx_PxPvdSceneClient* self__pod, PxPvdSceneFlags flags_pod) {
        physx::PxPvdSceneClient* self_ = reinterpret_cast<physx::PxPvdSceneClient*>(self__pod);
        auto flags = physx::PxPvdSceneFlags(flags_pod);
        self_->setScenePvdFlags(flags);
    }

    PxPvdSceneFlags PxPvdSceneClient_getScenePvdFlags(physx_PxPvdSceneClient const* self__pod) {
        physx::PxPvdSceneClient const* self_ = reinterpret_cast<physx::PxPvdSceneClient const*>(self__pod);
        physx::PxPvdSceneFlags return_val = self_->getScenePvdFlags();
        PxPvdSceneFlags return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxPvdSceneClient_updateCamera(physx_PxPvdSceneClient* self__pod, char const* name, physx_PxVec3 const* origin_pod, physx_PxVec3 const* up_pod, physx_PxVec3 const* target_pod) {
        physx::PxPvdSceneClient* self_ = reinterpret_cast<physx::PxPvdSceneClient*>(self__pod);
        physx::PxVec3 const& origin = reinterpret_cast<physx::PxVec3 const&>(*origin_pod);
        physx::PxVec3 const& up = reinterpret_cast<physx::PxVec3 const&>(*up_pod);
        physx::PxVec3 const& target = reinterpret_cast<physx::PxVec3 const&>(*target_pod);
        self_->updateCamera(name, origin, up, target);
    }

    void PxPvdSceneClient_drawPoints(physx_PxPvdSceneClient* self__pod, physx_PxDebugPoint const* points_pod, uint32_t count) {
        physx::PxPvdSceneClient* self_ = reinterpret_cast<physx::PxPvdSceneClient*>(self__pod);
        physx::PxDebugPoint const* points = reinterpret_cast<physx::PxDebugPoint const*>(points_pod);
        self_->drawPoints(points, count);
    }

    void PxPvdSceneClient_drawLines(physx_PxPvdSceneClient* self__pod, physx_PxDebugLine const* lines_pod, uint32_t count) {
        physx::PxPvdSceneClient* self_ = reinterpret_cast<physx::PxPvdSceneClient*>(self__pod);
        physx::PxDebugLine const* lines = reinterpret_cast<physx::PxDebugLine const*>(lines_pod);
        self_->drawLines(lines, count);
    }

    void PxPvdSceneClient_drawTriangles(physx_PxPvdSceneClient* self__pod, physx_PxDebugTriangle const* triangles_pod, uint32_t count) {
        physx::PxPvdSceneClient* self_ = reinterpret_cast<physx::PxPvdSceneClient*>(self__pod);
        physx::PxDebugTriangle const* triangles = reinterpret_cast<physx::PxDebugTriangle const*>(triangles_pod);
        self_->drawTriangles(triangles, count);
    }

    void PxPvdSceneClient_drawText(physx_PxPvdSceneClient* self__pod, physx_PxDebugText const* text_pod) {
        physx::PxPvdSceneClient* self_ = reinterpret_cast<physx::PxPvdSceneClient*>(self__pod);
        physx::PxDebugText const& text = reinterpret_cast<physx::PxDebugText const&>(*text_pod);
        self_->drawText(text);
    }

    physx_PxDominanceGroupPair PxDominanceGroupPair_new(uint8_t a, uint8_t b) {
        PxDominanceGroupPair return_val(a, b);
        physx_PxDominanceGroupPair return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxBroadPhaseCallback_delete(physx_PxBroadPhaseCallback* self__pod) {
        physx::PxBroadPhaseCallback* self_ = reinterpret_cast<physx::PxBroadPhaseCallback*>(self__pod);
        delete self_;
    }

    void PxBroadPhaseCallback_onObjectOutOfBounds(physx_PxBroadPhaseCallback* self__pod, physx_PxShape* shape_pod, physx_PxActor* actor_pod) {
        physx::PxBroadPhaseCallback* self_ = reinterpret_cast<physx::PxBroadPhaseCallback*>(self__pod);
        physx::PxShape& shape = reinterpret_cast<physx::PxShape&>(*shape_pod);
        physx::PxActor& actor = reinterpret_cast<physx::PxActor&>(*actor_pod);
        self_->onObjectOutOfBounds(shape, actor);
    }

    void PxBroadPhaseCallback_onObjectOutOfBounds_1(physx_PxBroadPhaseCallback* self__pod, physx_PxAggregate* aggregate_pod) {
        physx::PxBroadPhaseCallback* self_ = reinterpret_cast<physx::PxBroadPhaseCallback*>(self__pod);
        physx::PxAggregate& aggregate = reinterpret_cast<physx::PxAggregate&>(*aggregate_pod);
        self_->onObjectOutOfBounds(aggregate);
    }

    void PxScene_release(physx_PxScene* self__pod) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        self_->release();
    }

    void PxScene_setFlag(physx_PxScene* self__pod, PxSceneFlag flag_pod, bool value) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        auto flag = static_cast<physx::PxSceneFlag::Enum>(flag_pod);
        self_->setFlag(flag, value);
    }

    PxSceneFlags PxScene_getFlags(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        physx::PxSceneFlags return_val = self_->getFlags();
        PxSceneFlags return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxScene_setLimits(physx_PxScene* self__pod, physx_PxSceneLimits const* limits_pod) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxSceneLimits const& limits = reinterpret_cast<physx::PxSceneLimits const&>(*limits_pod);
        self_->setLimits(limits);
    }

    physx_PxSceneLimits PxScene_getLimits(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        physx::PxSceneLimits return_val = self_->getLimits();
        physx_PxSceneLimits return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxPhysics* PxScene_getPhysics(physx_PxScene* self__pod) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxPhysics& return_val = self_->getPhysics();
        auto return_val_pod = reinterpret_cast<physx_PxPhysics*>(&return_val);
        return return_val_pod;
    }

    uint32_t PxScene_getTimestamp(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        uint32_t return_val = self_->getTimestamp();
        return return_val;
    }

    bool PxScene_addArticulation(physx_PxScene* self__pod, physx_PxArticulationReducedCoordinate* articulation_pod) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxArticulationReducedCoordinate& articulation = reinterpret_cast<physx::PxArticulationReducedCoordinate&>(*articulation_pod);
        bool return_val = self_->addArticulation(articulation);
        return return_val;
    }

    void PxScene_removeArticulation(physx_PxScene* self__pod, physx_PxArticulationReducedCoordinate* articulation_pod, bool wakeOnLostTouch) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxArticulationReducedCoordinate& articulation = reinterpret_cast<physx::PxArticulationReducedCoordinate&>(*articulation_pod);
        self_->removeArticulation(articulation, wakeOnLostTouch);
    }

    bool PxScene_addActor(physx_PxScene* self__pod, physx_PxActor* actor_pod, physx_PxBVH const* bvh_pod) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxActor& actor = reinterpret_cast<physx::PxActor&>(*actor_pod);
        physx::PxBVH const* bvh = reinterpret_cast<physx::PxBVH const*>(bvh_pod);
        bool return_val = self_->addActor(actor, bvh);
        return return_val;
    }

    bool PxScene_addActors(physx_PxScene* self__pod, physx_PxActor* const* actors_pod, uint32_t nbActors) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxActor* const* actors = reinterpret_cast<physx::PxActor* const*>(actors_pod);
        bool return_val = self_->addActors(actors, nbActors);
        return return_val;
    }

    bool PxScene_addActors_1(physx_PxScene* self__pod, physx_PxPruningStructure const* pruningStructure_pod) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxPruningStructure const& pruningStructure = reinterpret_cast<physx::PxPruningStructure const&>(*pruningStructure_pod);
        bool return_val = self_->addActors(pruningStructure);
        return return_val;
    }

    void PxScene_removeActor(physx_PxScene* self__pod, physx_PxActor* actor_pod, bool wakeOnLostTouch) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxActor& actor = reinterpret_cast<physx::PxActor&>(*actor_pod);
        self_->removeActor(actor, wakeOnLostTouch);
    }

    void PxScene_removeActors(physx_PxScene* self__pod, physx_PxActor* const* actors_pod, uint32_t nbActors, bool wakeOnLostTouch) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxActor* const* actors = reinterpret_cast<physx::PxActor* const*>(actors_pod);
        self_->removeActors(actors, nbActors, wakeOnLostTouch);
    }

    bool PxScene_addAggregate(physx_PxScene* self__pod, physx_PxAggregate* aggregate_pod) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxAggregate& aggregate = reinterpret_cast<physx::PxAggregate&>(*aggregate_pod);
        bool return_val = self_->addAggregate(aggregate);
        return return_val;
    }

    void PxScene_removeAggregate(physx_PxScene* self__pod, physx_PxAggregate* aggregate_pod, bool wakeOnLostTouch) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxAggregate& aggregate = reinterpret_cast<physx::PxAggregate&>(*aggregate_pod);
        self_->removeAggregate(aggregate, wakeOnLostTouch);
    }

    bool PxScene_addCollection(physx_PxScene* self__pod, physx_PxCollection const* collection_pod) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxCollection const& collection = reinterpret_cast<physx::PxCollection const&>(*collection_pod);
        bool return_val = self_->addCollection(collection);
        return return_val;
    }

    uint32_t PxScene_getNbActors(physx_PxScene const* self__pod, PxActorTypeFlags types_pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        auto types = physx::PxActorTypeFlags(types_pod);
        uint32_t return_val = self_->getNbActors(types);
        return return_val;
    }

    uint32_t PxScene_getActors(physx_PxScene const* self__pod, PxActorTypeFlags types_pod, physx_PxActor** userBuffer_pod, uint32_t bufferSize, uint32_t startIndex) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        auto types = physx::PxActorTypeFlags(types_pod);
        physx::PxActor** userBuffer = reinterpret_cast<physx::PxActor**>(userBuffer_pod);
        uint32_t return_val = self_->getActors(types, userBuffer, bufferSize, startIndex);
        return return_val;
    }

    physx_PxActor** PxScene_getActiveActors(physx_PxScene* self__pod, uint32_t* nbActorsOut_pod) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        uint32_t& nbActorsOut = *nbActorsOut_pod;
        physx::PxActor** return_val = self_->getActiveActors(nbActorsOut);
        auto return_val_pod = reinterpret_cast<physx_PxActor**>(return_val);
        return return_val_pod;
    }

    uint32_t PxScene_getNbArticulations(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        uint32_t return_val = self_->getNbArticulations();
        return return_val;
    }

    uint32_t PxScene_getArticulations(physx_PxScene const* self__pod, physx_PxArticulationReducedCoordinate** userBuffer_pod, uint32_t bufferSize, uint32_t startIndex) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        physx::PxArticulationReducedCoordinate** userBuffer = reinterpret_cast<physx::PxArticulationReducedCoordinate**>(userBuffer_pod);
        uint32_t return_val = self_->getArticulations(userBuffer, bufferSize, startIndex);
        return return_val;
    }

    uint32_t PxScene_getNbConstraints(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        uint32_t return_val = self_->getNbConstraints();
        return return_val;
    }

    uint32_t PxScene_getConstraints(physx_PxScene const* self__pod, physx_PxConstraint** userBuffer_pod, uint32_t bufferSize, uint32_t startIndex) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        physx::PxConstraint** userBuffer = reinterpret_cast<physx::PxConstraint**>(userBuffer_pod);
        uint32_t return_val = self_->getConstraints(userBuffer, bufferSize, startIndex);
        return return_val;
    }

    uint32_t PxScene_getNbAggregates(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        uint32_t return_val = self_->getNbAggregates();
        return return_val;
    }

    uint32_t PxScene_getAggregates(physx_PxScene const* self__pod, physx_PxAggregate** userBuffer_pod, uint32_t bufferSize, uint32_t startIndex) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        physx::PxAggregate** userBuffer = reinterpret_cast<physx::PxAggregate**>(userBuffer_pod);
        uint32_t return_val = self_->getAggregates(userBuffer, bufferSize, startIndex);
        return return_val;
    }

    void PxScene_setDominanceGroupPair(physx_PxScene* self__pod, uint8_t group1, uint8_t group2, physx_PxDominanceGroupPair const* dominance_pod) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxDominanceGroupPair const& dominance = reinterpret_cast<physx::PxDominanceGroupPair const&>(*dominance_pod);
        self_->setDominanceGroupPair(group1, group2, dominance);
    }

    physx_PxDominanceGroupPair PxScene_getDominanceGroupPair(physx_PxScene const* self__pod, uint8_t group1, uint8_t group2) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        physx::PxDominanceGroupPair return_val = self_->getDominanceGroupPair(group1, group2);
        physx_PxDominanceGroupPair return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxCpuDispatcher* PxScene_getCpuDispatcher(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        physx::PxCpuDispatcher* return_val = self_->getCpuDispatcher();
        auto return_val_pod = reinterpret_cast<physx_PxCpuDispatcher*>(return_val);
        return return_val_pod;
    }

    uint8_t PxScene_createClient(physx_PxScene* self__pod) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        uint8_t return_val = self_->createClient();
        return return_val;
    }

    void PxScene_setSimulationEventCallback(physx_PxScene* self__pod, physx_PxSimulationEventCallback* callback_pod) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxSimulationEventCallback* callback = reinterpret_cast<physx::PxSimulationEventCallback*>(callback_pod);
        self_->setSimulationEventCallback(callback);
    }

    physx_PxSimulationEventCallback* PxScene_getSimulationEventCallback(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        physx::PxSimulationEventCallback* return_val = self_->getSimulationEventCallback();
        auto return_val_pod = reinterpret_cast<physx_PxSimulationEventCallback*>(return_val);
        return return_val_pod;
    }

    void PxScene_setContactModifyCallback(physx_PxScene* self__pod, physx_PxContactModifyCallback* callback_pod) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxContactModifyCallback* callback = reinterpret_cast<physx::PxContactModifyCallback*>(callback_pod);
        self_->setContactModifyCallback(callback);
    }

    void PxScene_setCCDContactModifyCallback(physx_PxScene* self__pod, physx_PxCCDContactModifyCallback* callback_pod) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxCCDContactModifyCallback* callback = reinterpret_cast<physx::PxCCDContactModifyCallback*>(callback_pod);
        self_->setCCDContactModifyCallback(callback);
    }

    physx_PxContactModifyCallback* PxScene_getContactModifyCallback(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        physx::PxContactModifyCallback* return_val = self_->getContactModifyCallback();
        auto return_val_pod = reinterpret_cast<physx_PxContactModifyCallback*>(return_val);
        return return_val_pod;
    }

    physx_PxCCDContactModifyCallback* PxScene_getCCDContactModifyCallback(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        physx::PxCCDContactModifyCallback* return_val = self_->getCCDContactModifyCallback();
        auto return_val_pod = reinterpret_cast<physx_PxCCDContactModifyCallback*>(return_val);
        return return_val_pod;
    }

    void PxScene_setBroadPhaseCallback(physx_PxScene* self__pod, physx_PxBroadPhaseCallback* callback_pod) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxBroadPhaseCallback* callback = reinterpret_cast<physx::PxBroadPhaseCallback*>(callback_pod);
        self_->setBroadPhaseCallback(callback);
    }

    physx_PxBroadPhaseCallback* PxScene_getBroadPhaseCallback(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        physx::PxBroadPhaseCallback* return_val = self_->getBroadPhaseCallback();
        auto return_val_pod = reinterpret_cast<physx_PxBroadPhaseCallback*>(return_val);
        return return_val_pod;
    }

    void PxScene_setFilterShaderData(physx_PxScene* self__pod, void const* data, uint32_t dataSize) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        self_->setFilterShaderData(data, dataSize);
    }

    void const* PxScene_getFilterShaderData(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        void const* return_val = self_->getFilterShaderData();
        return return_val;
    }

    uint32_t PxScene_getFilterShaderDataSize(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        uint32_t return_val = self_->getFilterShaderDataSize();
        return return_val;
    }

    bool PxScene_resetFiltering(physx_PxScene* self__pod, physx_PxActor* actor_pod) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxActor& actor = reinterpret_cast<physx::PxActor&>(*actor_pod);
        bool return_val = self_->resetFiltering(actor);
        return return_val;
    }

    bool PxScene_resetFiltering_1(physx_PxScene* self__pod, physx_PxRigidActor* actor_pod, physx_PxShape* const* shapes_pod, uint32_t shapeCount) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxRigidActor& actor = reinterpret_cast<physx::PxRigidActor&>(*actor_pod);
        physx::PxShape* const* shapes = reinterpret_cast<physx::PxShape* const*>(shapes_pod);
        bool return_val = self_->resetFiltering(actor, shapes, shapeCount);
        return return_val;
    }

    PxPairFilteringMode PxScene_getKinematicKinematicFilteringMode(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        physx::PxPairFilteringMode::Enum return_val = self_->getKinematicKinematicFilteringMode();
        PxPairFilteringMode return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    PxPairFilteringMode PxScene_getStaticKinematicFilteringMode(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        physx::PxPairFilteringMode::Enum return_val = self_->getStaticKinematicFilteringMode();
        PxPairFilteringMode return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxScene_simulate(physx_PxScene* self__pod, float elapsedTime, physx_PxBaseTask* completionTask_pod, void* scratchMemBlock, uint32_t scratchMemBlockSize, bool controlSimulation) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxBaseTask* completionTask = reinterpret_cast<physx::PxBaseTask*>(completionTask_pod);
        bool return_val = self_->simulate(elapsedTime, completionTask, scratchMemBlock, scratchMemBlockSize, controlSimulation);
        return return_val;
    }

    bool PxScene_advance(physx_PxScene* self__pod, physx_PxBaseTask* completionTask_pod) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxBaseTask* completionTask = reinterpret_cast<physx::PxBaseTask*>(completionTask_pod);
        bool return_val = self_->advance(completionTask);
        return return_val;
    }

    bool PxScene_collide(physx_PxScene* self__pod, float elapsedTime, physx_PxBaseTask* completionTask_pod, void* scratchMemBlock, uint32_t scratchMemBlockSize, bool controlSimulation) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxBaseTask* completionTask = reinterpret_cast<physx::PxBaseTask*>(completionTask_pod);
        bool return_val = self_->collide(elapsedTime, completionTask, scratchMemBlock, scratchMemBlockSize, controlSimulation);
        return return_val;
    }

    bool PxScene_checkResults(physx_PxScene* self__pod, bool block) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        bool return_val = self_->checkResults(block);
        return return_val;
    }

    bool PxScene_fetchCollision(physx_PxScene* self__pod, bool block) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        bool return_val = self_->fetchCollision(block);
        return return_val;
    }

    bool PxScene_fetchResults(physx_PxScene* self__pod, bool block, uint32_t* errorState) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        bool return_val = self_->fetchResults(block, errorState);
        return return_val;
    }

    bool PxScene_fetchResultsStart(physx_PxScene* self__pod, physx_PxContactPairHeader const** contactPairs_pod, uint32_t* nbContactPairs_pod, bool block) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxContactPairHeader const*& contactPairs = reinterpret_cast<physx::PxContactPairHeader const*&>(*contactPairs_pod);
        uint32_t& nbContactPairs = *nbContactPairs_pod;
        bool return_val = self_->fetchResultsStart(contactPairs, nbContactPairs, block);
        return return_val;
    }

    void PxScene_processCallbacks(physx_PxScene* self__pod, physx_PxBaseTask* continuation_pod) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxBaseTask* continuation = reinterpret_cast<physx::PxBaseTask*>(continuation_pod);
        self_->processCallbacks(continuation);
    }

    void PxScene_fetchResultsFinish(physx_PxScene* self__pod, uint32_t* errorState) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        self_->fetchResultsFinish(errorState);
    }

    void PxScene_fetchResultsParticleSystem(physx_PxScene* self__pod) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        self_->fetchResultsParticleSystem();
    }

    void PxScene_flushSimulation(physx_PxScene* self__pod, bool sendPendingReports) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        self_->flushSimulation(sendPendingReports);
    }

    void PxScene_setGravity(physx_PxScene* self__pod, physx_PxVec3 const* vec_pod) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxVec3 const& vec = reinterpret_cast<physx::PxVec3 const&>(*vec_pod);
        self_->setGravity(vec);
    }

    physx_PxVec3 PxScene_getGravity(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        physx::PxVec3 return_val = self_->getGravity();
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxScene_setBounceThresholdVelocity(physx_PxScene* self__pod, float t) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        self_->setBounceThresholdVelocity(t);
    }

    float PxScene_getBounceThresholdVelocity(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        float return_val = self_->getBounceThresholdVelocity();
        return return_val;
    }

    void PxScene_setCCDMaxPasses(physx_PxScene* self__pod, uint32_t ccdMaxPasses) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        self_->setCCDMaxPasses(ccdMaxPasses);
    }

    uint32_t PxScene_getCCDMaxPasses(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        uint32_t return_val = self_->getCCDMaxPasses();
        return return_val;
    }

    void PxScene_setCCDMaxSeparation(physx_PxScene* self__pod, float t) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        self_->setCCDMaxSeparation(t);
    }

    float PxScene_getCCDMaxSeparation(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        float return_val = self_->getCCDMaxSeparation();
        return return_val;
    }

    void PxScene_setCCDThreshold(physx_PxScene* self__pod, float t) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        self_->setCCDThreshold(t);
    }

    float PxScene_getCCDThreshold(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        float return_val = self_->getCCDThreshold();
        return return_val;
    }

    void PxScene_setMaxBiasCoefficient(physx_PxScene* self__pod, float t) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        self_->setMaxBiasCoefficient(t);
    }

    float PxScene_getMaxBiasCoefficient(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        float return_val = self_->getMaxBiasCoefficient();
        return return_val;
    }

    void PxScene_setFrictionOffsetThreshold(physx_PxScene* self__pod, float t) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        self_->setFrictionOffsetThreshold(t);
    }

    float PxScene_getFrictionOffsetThreshold(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        float return_val = self_->getFrictionOffsetThreshold();
        return return_val;
    }

    void PxScene_setFrictionCorrelationDistance(physx_PxScene* self__pod, float t) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        self_->setFrictionCorrelationDistance(t);
    }

    float PxScene_getFrictionCorrelationDistance(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        float return_val = self_->getFrictionCorrelationDistance();
        return return_val;
    }

    PxFrictionType PxScene_getFrictionType(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        physx::PxFrictionType::Enum return_val = self_->getFrictionType();
        PxFrictionType return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    PxSolverType PxScene_getSolverType(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        physx::PxSolverType::Enum return_val = self_->getSolverType();
        PxSolverType return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxScene_setVisualizationParameter(physx_PxScene* self__pod, PxVisualizationParameter param_pod, float value) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        auto param = static_cast<physx::PxVisualizationParameter::Enum>(param_pod);
        bool return_val = self_->setVisualizationParameter(param, value);
        return return_val;
    }

    float PxScene_getVisualizationParameter(physx_PxScene const* self__pod, PxVisualizationParameter paramEnum_pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        auto paramEnum = static_cast<physx::PxVisualizationParameter::Enum>(paramEnum_pod);
        float return_val = self_->getVisualizationParameter(paramEnum);
        return return_val;
    }

    void PxScene_setVisualizationCullingBox(physx_PxScene* self__pod, physx_PxBounds3 const* box_pod) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxBounds3 const& box = reinterpret_cast<physx::PxBounds3 const&>(*box_pod);
        self_->setVisualizationCullingBox(box);
    }

    physx_PxBounds3 PxScene_getVisualizationCullingBox(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        physx::PxBounds3 return_val = self_->getVisualizationCullingBox();
        physx_PxBounds3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxRenderBuffer const* PxScene_getRenderBuffer(physx_PxScene* self__pod) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxRenderBuffer const& return_val = self_->getRenderBuffer();
        auto return_val_pod = reinterpret_cast<physx_PxRenderBuffer const*>(&return_val);
        return return_val_pod;
    }

    void PxScene_getSimulationStatistics(physx_PxScene const* self__pod, physx_PxSimulationStatistics* stats_pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        physx::PxSimulationStatistics& stats = reinterpret_cast<physx::PxSimulationStatistics&>(*stats_pod);
        self_->getSimulationStatistics(stats);
    }

    PxBroadPhaseType PxScene_getBroadPhaseType(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        physx::PxBroadPhaseType::Enum return_val = self_->getBroadPhaseType();
        PxBroadPhaseType return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxScene_getBroadPhaseCaps(physx_PxScene const* self__pod, physx_PxBroadPhaseCaps* caps_pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        physx::PxBroadPhaseCaps& caps = reinterpret_cast<physx::PxBroadPhaseCaps&>(*caps_pod);
        bool return_val = self_->getBroadPhaseCaps(caps);
        return return_val;
    }

    uint32_t PxScene_getNbBroadPhaseRegions(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        uint32_t return_val = self_->getNbBroadPhaseRegions();
        return return_val;
    }

    uint32_t PxScene_getBroadPhaseRegions(physx_PxScene const* self__pod, physx_PxBroadPhaseRegionInfo* userBuffer_pod, uint32_t bufferSize, uint32_t startIndex) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        physx::PxBroadPhaseRegionInfo* userBuffer = reinterpret_cast<physx::PxBroadPhaseRegionInfo*>(userBuffer_pod);
        uint32_t return_val = self_->getBroadPhaseRegions(userBuffer, bufferSize, startIndex);
        return return_val;
    }

    uint32_t PxScene_addBroadPhaseRegion(physx_PxScene* self__pod, physx_PxBroadPhaseRegion const* region_pod, bool populateRegion) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxBroadPhaseRegion const& region = reinterpret_cast<physx::PxBroadPhaseRegion const&>(*region_pod);
        uint32_t return_val = self_->addBroadPhaseRegion(region, populateRegion);
        return return_val;
    }

    bool PxScene_removeBroadPhaseRegion(physx_PxScene* self__pod, uint32_t handle) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        bool return_val = self_->removeBroadPhaseRegion(handle);
        return return_val;
    }

    physx_PxTaskManager* PxScene_getTaskManager(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        physx::PxTaskManager* return_val = self_->getTaskManager();
        auto return_val_pod = reinterpret_cast<physx_PxTaskManager*>(return_val);
        return return_val_pod;
    }

    void PxScene_lockRead(physx_PxScene* self__pod, char const* file, uint32_t line) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        self_->lockRead(file, line);
    }

    void PxScene_unlockRead(physx_PxScene* self__pod) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        self_->unlockRead();
    }

    void PxScene_lockWrite(physx_PxScene* self__pod, char const* file, uint32_t line) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        self_->lockWrite(file, line);
    }

    void PxScene_unlockWrite(physx_PxScene* self__pod) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        self_->unlockWrite();
    }

    void PxScene_setNbContactDataBlocks(physx_PxScene* self__pod, uint32_t numBlocks) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        self_->setNbContactDataBlocks(numBlocks);
    }

    uint32_t PxScene_getNbContactDataBlocksUsed(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        uint32_t return_val = self_->getNbContactDataBlocksUsed();
        return return_val;
    }

    uint32_t PxScene_getMaxNbContactDataBlocksUsed(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        uint32_t return_val = self_->getMaxNbContactDataBlocksUsed();
        return return_val;
    }

    uint32_t PxScene_getContactReportStreamBufferSize(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        uint32_t return_val = self_->getContactReportStreamBufferSize();
        return return_val;
    }

    void PxScene_setSolverBatchSize(physx_PxScene* self__pod, uint32_t solverBatchSize) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        self_->setSolverBatchSize(solverBatchSize);
    }

    uint32_t PxScene_getSolverBatchSize(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        uint32_t return_val = self_->getSolverBatchSize();
        return return_val;
    }

    void PxScene_setSolverArticulationBatchSize(physx_PxScene* self__pod, uint32_t solverBatchSize) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        self_->setSolverArticulationBatchSize(solverBatchSize);
    }

    uint32_t PxScene_getSolverArticulationBatchSize(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        uint32_t return_val = self_->getSolverArticulationBatchSize();
        return return_val;
    }

    float PxScene_getWakeCounterResetValue(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        float return_val = self_->getWakeCounterResetValue();
        return return_val;
    }

    void PxScene_shiftOrigin(physx_PxScene* self__pod, physx_PxVec3 const* shift_pod) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxVec3 const& shift = reinterpret_cast<physx::PxVec3 const&>(*shift_pod);
        self_->shiftOrigin(shift);
    }

    physx_PxPvdSceneClient* PxScene_getScenePvdClient(physx_PxScene* self__pod) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxPvdSceneClient* return_val = self_->getScenePvdClient();
        auto return_val_pod = reinterpret_cast<physx_PxPvdSceneClient*>(return_val);
        return return_val_pod;
    }

    void PxScene_copyArticulationData(physx_PxScene* self__pod, void* data, void* index, PxArticulationGpuDataType dataType_pod, uint32_t nbCopyArticulations, void* copyEvent) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        auto dataType = static_cast<physx::PxArticulationGpuDataType::Enum>(dataType_pod);
        self_->copyArticulationData(data, index, dataType, nbCopyArticulations, copyEvent);
    }

    void PxScene_applyArticulationData(physx_PxScene* self__pod, void* data, void* index, PxArticulationGpuDataType dataType_pod, uint32_t nbUpdatedArticulations, void* waitEvent, void* signalEvent) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        auto dataType = static_cast<physx::PxArticulationGpuDataType::Enum>(dataType_pod);
        self_->applyArticulationData(data, index, dataType, nbUpdatedArticulations, waitEvent, signalEvent);
    }

    void PxScene_copySoftBodyData(physx_PxScene* self__pod, void** data, void* dataSizes, void* softBodyIndices, PxSoftBodyDataFlag flag_pod, uint32_t nbCopySoftBodies, uint32_t maxSize, void* copyEvent) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        auto flag = static_cast<physx::PxSoftBodyDataFlag::Enum>(flag_pod);
        self_->copySoftBodyData(data, dataSizes, softBodyIndices, flag, nbCopySoftBodies, maxSize, copyEvent);
    }

    void PxScene_applySoftBodyData(physx_PxScene* self__pod, void** data, void* dataSizes, void* softBodyIndices, PxSoftBodyDataFlag flag_pod, uint32_t nbUpdatedSoftBodies, uint32_t maxSize, void* applyEvent) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        auto flag = static_cast<physx::PxSoftBodyDataFlag::Enum>(flag_pod);
        self_->applySoftBodyData(data, dataSizes, softBodyIndices, flag, nbUpdatedSoftBodies, maxSize, applyEvent);
    }

    void PxScene_copyContactData(physx_PxScene* self__pod, void* data, uint32_t maxContactPairs, void* numContactPairs, void* copyEvent) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        self_->copyContactData(data, maxContactPairs, numContactPairs, copyEvent);
    }

    void PxScene_copyBodyData(physx_PxScene* self__pod, physx_PxGpuBodyData* data_pod, physx_PxGpuActorPair* index_pod, uint32_t nbCopyActors, void* copyEvent) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxGpuBodyData* data = reinterpret_cast<physx::PxGpuBodyData*>(data_pod);
        physx::PxGpuActorPair* index = reinterpret_cast<physx::PxGpuActorPair*>(index_pod);
        self_->copyBodyData(data, index, nbCopyActors, copyEvent);
    }

    void PxScene_applyActorData(physx_PxScene* self__pod, void* data, physx_PxGpuActorPair* index_pod, PxActorCacheFlag flag_pod, uint32_t nbUpdatedActors, void* waitEvent, void* signalEvent) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxGpuActorPair* index = reinterpret_cast<physx::PxGpuActorPair*>(index_pod);
        auto flag = static_cast<physx::PxActorCacheFlag::Enum>(flag_pod);
        self_->applyActorData(data, index, flag, nbUpdatedActors, waitEvent, signalEvent);
    }

    void PxScene_computeDenseJacobians(physx_PxScene* self__pod, physx_PxIndexDataPair const* indices_pod, uint32_t nbIndices, void* computeEvent) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxIndexDataPair const* indices = reinterpret_cast<physx::PxIndexDataPair const*>(indices_pod);
        self_->computeDenseJacobians(indices, nbIndices, computeEvent);
    }

    void PxScene_computeGeneralizedMassMatrices(physx_PxScene* self__pod, physx_PxIndexDataPair const* indices_pod, uint32_t nbIndices, void* computeEvent) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxIndexDataPair const* indices = reinterpret_cast<physx::PxIndexDataPair const*>(indices_pod);
        self_->computeGeneralizedMassMatrices(indices, nbIndices, computeEvent);
    }

    void PxScene_computeGeneralizedGravityForces(physx_PxScene* self__pod, physx_PxIndexDataPair const* indices_pod, uint32_t nbIndices, void* computeEvent) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxIndexDataPair const* indices = reinterpret_cast<physx::PxIndexDataPair const*>(indices_pod);
        self_->computeGeneralizedGravityForces(indices, nbIndices, computeEvent);
    }

    void PxScene_computeCoriolisAndCentrifugalForces(physx_PxScene* self__pod, physx_PxIndexDataPair const* indices_pod, uint32_t nbIndices, void* computeEvent) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxIndexDataPair const* indices = reinterpret_cast<physx::PxIndexDataPair const*>(indices_pod);
        self_->computeCoriolisAndCentrifugalForces(indices, nbIndices, computeEvent);
    }

    physx_PxgDynamicsMemoryConfig PxScene_getGpuDynamicsConfig(physx_PxScene const* self__pod) {
        physx::PxScene const* self_ = reinterpret_cast<physx::PxScene const*>(self__pod);
        physx::PxgDynamicsMemoryConfig return_val = self_->getGpuDynamicsConfig();
        physx_PxgDynamicsMemoryConfig return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxScene_applyParticleBufferData(physx_PxScene* self__pod, uint32_t const* indices, physx_PxGpuParticleBufferIndexPair const* bufferIndexPair_pod, PxParticleBufferFlags const* flags_pod, uint32_t nbUpdatedBuffers, void* waitEvent, void* signalEvent) {
        physx::PxScene* self_ = reinterpret_cast<physx::PxScene*>(self__pod);
        physx::PxGpuParticleBufferIndexPair const* bufferIndexPair = reinterpret_cast<physx::PxGpuParticleBufferIndexPair const*>(bufferIndexPair_pod);
        physx::PxParticleBufferFlags const* flags = reinterpret_cast<physx::PxParticleBufferFlags const*>(flags_pod);
        self_->applyParticleBufferData(indices, bufferIndexPair, flags, nbUpdatedBuffers, waitEvent, signalEvent);
    }

    physx_PxSceneReadLock* PxSceneReadLock_new_alloc(physx_PxScene* scene_pod, char const* file, uint32_t line) {
        physx::PxScene& scene = reinterpret_cast<physx::PxScene&>(*scene_pod);
        auto return_val = new physx::PxSceneReadLock(scene, file, line);
        auto return_val_pod = reinterpret_cast<physx_PxSceneReadLock*>(return_val);
        return return_val_pod;
    }

    void PxSceneReadLock_delete(physx_PxSceneReadLock* self__pod) {
        physx::PxSceneReadLock* self_ = reinterpret_cast<physx::PxSceneReadLock*>(self__pod);
        delete self_;
    }

    physx_PxSceneWriteLock* PxSceneWriteLock_new_alloc(physx_PxScene* scene_pod, char const* file, uint32_t line) {
        physx::PxScene& scene = reinterpret_cast<physx::PxScene&>(*scene_pod);
        auto return_val = new physx::PxSceneWriteLock(scene, file, line);
        auto return_val_pod = reinterpret_cast<physx_PxSceneWriteLock*>(return_val);
        return return_val_pod;
    }

    void PxSceneWriteLock_delete(physx_PxSceneWriteLock* self__pod) {
        physx::PxSceneWriteLock* self_ = reinterpret_cast<physx::PxSceneWriteLock*>(self__pod);
        delete self_;
    }

    physx_PxContactPairExtraDataItem PxContactPairExtraDataItem_new() {
        PxContactPairExtraDataItem return_val;
        physx_PxContactPairExtraDataItem return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxContactPairVelocity PxContactPairVelocity_new() {
        PxContactPairVelocity return_val;
        physx_PxContactPairVelocity return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxContactPairPose PxContactPairPose_new() {
        PxContactPairPose return_val;
        physx_PxContactPairPose return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxContactPairIndex PxContactPairIndex_new() {
        PxContactPairIndex return_val;
        physx_PxContactPairIndex return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxContactPairExtraDataIterator PxContactPairExtraDataIterator_new(uint8_t const* stream, uint32_t size) {
        PxContactPairExtraDataIterator return_val(stream, size);
        physx_PxContactPairExtraDataIterator return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxContactPairExtraDataIterator_nextItemSet(physx_PxContactPairExtraDataIterator* self__pod) {
        physx::PxContactPairExtraDataIterator* self_ = reinterpret_cast<physx::PxContactPairExtraDataIterator*>(self__pod);
        bool return_val = self_->nextItemSet();
        return return_val;
    }

    physx_PxContactPairHeader PxContactPairHeader_new() {
        PxContactPairHeader return_val;
        physx_PxContactPairHeader return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxContactPair PxContactPair_new() {
        PxContactPair return_val;
        physx_PxContactPair return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    uint32_t PxContactPair_extractContacts(physx_PxContactPair const* self__pod, physx_PxContactPairPoint* userBuffer_pod, uint32_t bufferSize) {
        physx::PxContactPair const* self_ = reinterpret_cast<physx::PxContactPair const*>(self__pod);
        physx::PxContactPairPoint* userBuffer = reinterpret_cast<physx::PxContactPairPoint*>(userBuffer_pod);
        uint32_t return_val = self_->extractContacts(userBuffer, bufferSize);
        return return_val;
    }

    void PxContactPair_bufferContacts(physx_PxContactPair const* self__pod, physx_PxContactPair* newPair_pod, uint8_t* bufferMemory) {
        physx::PxContactPair const* self_ = reinterpret_cast<physx::PxContactPair const*>(self__pod);
        physx::PxContactPair* newPair = reinterpret_cast<physx::PxContactPair*>(newPair_pod);
        self_->bufferContacts(newPair, bufferMemory);
    }

    uint32_t const* PxContactPair_getInternalFaceIndices(physx_PxContactPair const* self__pod) {
        physx::PxContactPair const* self_ = reinterpret_cast<physx::PxContactPair const*>(self__pod);
        uint32_t const* return_val = self_->getInternalFaceIndices();
        return return_val;
    }

    physx_PxTriggerPair PxTriggerPair_new() {
        PxTriggerPair return_val;
        physx_PxTriggerPair return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxConstraintInfo PxConstraintInfo_new() {
        PxConstraintInfo return_val;
        physx_PxConstraintInfo return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxConstraintInfo PxConstraintInfo_new_1(physx_PxConstraint* c_pod, void* extRef, uint32_t t) {
        physx::PxConstraint* c = reinterpret_cast<physx::PxConstraint*>(c_pod);
        PxConstraintInfo return_val(c, extRef, t);
        physx_PxConstraintInfo return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxSimulationEventCallback_onConstraintBreak(physx_PxSimulationEventCallback* self__pod, physx_PxConstraintInfo* constraints_pod, uint32_t count) {
        physx::PxSimulationEventCallback* self_ = reinterpret_cast<physx::PxSimulationEventCallback*>(self__pod);
        physx::PxConstraintInfo* constraints = reinterpret_cast<physx::PxConstraintInfo*>(constraints_pod);
        self_->onConstraintBreak(constraints, count);
    }

    void PxSimulationEventCallback_onWake(physx_PxSimulationEventCallback* self__pod, physx_PxActor** actors_pod, uint32_t count) {
        physx::PxSimulationEventCallback* self_ = reinterpret_cast<physx::PxSimulationEventCallback*>(self__pod);
        physx::PxActor** actors = reinterpret_cast<physx::PxActor**>(actors_pod);
        self_->onWake(actors, count);
    }

    void PxSimulationEventCallback_onSleep(physx_PxSimulationEventCallback* self__pod, physx_PxActor** actors_pod, uint32_t count) {
        physx::PxSimulationEventCallback* self_ = reinterpret_cast<physx::PxSimulationEventCallback*>(self__pod);
        physx::PxActor** actors = reinterpret_cast<physx::PxActor**>(actors_pod);
        self_->onSleep(actors, count);
    }

    void PxSimulationEventCallback_onContact(physx_PxSimulationEventCallback* self__pod, physx_PxContactPairHeader const* pairHeader_pod, physx_PxContactPair const* pairs_pod, uint32_t nbPairs) {
        physx::PxSimulationEventCallback* self_ = reinterpret_cast<physx::PxSimulationEventCallback*>(self__pod);
        physx::PxContactPairHeader const& pairHeader = reinterpret_cast<physx::PxContactPairHeader const&>(*pairHeader_pod);
        physx::PxContactPair const* pairs = reinterpret_cast<physx::PxContactPair const*>(pairs_pod);
        self_->onContact(pairHeader, pairs, nbPairs);
    }

    void PxSimulationEventCallback_onTrigger(physx_PxSimulationEventCallback* self__pod, physx_PxTriggerPair* pairs_pod, uint32_t count) {
        physx::PxSimulationEventCallback* self_ = reinterpret_cast<physx::PxSimulationEventCallback*>(self__pod);
        physx::PxTriggerPair* pairs = reinterpret_cast<physx::PxTriggerPair*>(pairs_pod);
        self_->onTrigger(pairs, count);
    }

    void PxSimulationEventCallback_onAdvance(physx_PxSimulationEventCallback* self__pod, physx_PxRigidBody const* const* bodyBuffer_pod, physx_PxTransform const* poseBuffer_pod, uint32_t count) {
        physx::PxSimulationEventCallback* self_ = reinterpret_cast<physx::PxSimulationEventCallback*>(self__pod);
        physx::PxRigidBody const* const* bodyBuffer = reinterpret_cast<physx::PxRigidBody const* const*>(bodyBuffer_pod);
        physx::PxTransform const* poseBuffer = reinterpret_cast<physx::PxTransform const*>(poseBuffer_pod);
        self_->onAdvance(bodyBuffer, poseBuffer, count);
    }

    void PxSimulationEventCallback_delete(physx_PxSimulationEventCallback* self__pod) {
        physx::PxSimulationEventCallback* self_ = reinterpret_cast<physx::PxSimulationEventCallback*>(self__pod);
        delete self_;
    }

    physx_PxFEMParameters PxFEMParameters_new() {
        PxFEMParameters return_val;
        physx_PxFEMParameters return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxPruningStructure_release(physx_PxPruningStructure* self__pod) {
        physx::PxPruningStructure* self_ = reinterpret_cast<physx::PxPruningStructure*>(self__pod);
        self_->release();
    }

    uint32_t PxPruningStructure_getRigidActors(physx_PxPruningStructure const* self__pod, physx_PxRigidActor** userBuffer_pod, uint32_t bufferSize, uint32_t startIndex) {
        physx::PxPruningStructure const* self_ = reinterpret_cast<physx::PxPruningStructure const*>(self__pod);
        physx::PxRigidActor** userBuffer = reinterpret_cast<physx::PxRigidActor**>(userBuffer_pod);
        uint32_t return_val = self_->getRigidActors(userBuffer, bufferSize, startIndex);
        return return_val;
    }

    uint32_t PxPruningStructure_getNbRigidActors(physx_PxPruningStructure const* self__pod) {
        physx::PxPruningStructure const* self_ = reinterpret_cast<physx::PxPruningStructure const*>(self__pod);
        uint32_t return_val = self_->getNbRigidActors();
        return return_val;
    }

    void const* PxPruningStructure_getStaticMergeData(physx_PxPruningStructure const* self__pod) {
        physx::PxPruningStructure const* self_ = reinterpret_cast<physx::PxPruningStructure const*>(self__pod);
        void const* return_val = self_->getStaticMergeData();
        return return_val;
    }

    void const* PxPruningStructure_getDynamicMergeData(physx_PxPruningStructure const* self__pod) {
        physx::PxPruningStructure const* self_ = reinterpret_cast<physx::PxPruningStructure const*>(self__pod);
        void const* return_val = self_->getDynamicMergeData();
        return return_val;
    }

    char const* PxPruningStructure_getConcreteTypeName(physx_PxPruningStructure const* self__pod) {
        physx::PxPruningStructure const* self_ = reinterpret_cast<physx::PxPruningStructure const*>(self__pod);
        char const* return_val = self_->getConcreteTypeName();
        return return_val;
    }

    physx_PxExtendedVec3 PxExtendedVec3_new() {
        PxExtendedVec3 return_val;
        physx_PxExtendedVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxExtendedVec3 PxExtendedVec3_new_1(double _x, double _y, double _z) {
        PxExtendedVec3 return_val(_x, _y, _z);
        physx_PxExtendedVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxExtendedVec3_isZero(physx_PxExtendedVec3 const* self__pod) {
        physx::PxExtendedVec3 const* self_ = reinterpret_cast<physx::PxExtendedVec3 const*>(self__pod);
        bool return_val = self_->isZero();
        return return_val;
    }

    double PxExtendedVec3_dot(physx_PxExtendedVec3 const* self__pod, physx_PxVec3 const* v_pod) {
        physx::PxExtendedVec3 const* self_ = reinterpret_cast<physx::PxExtendedVec3 const*>(self__pod);
        physx::PxVec3 const& v = reinterpret_cast<physx::PxVec3 const&>(*v_pod);
        double return_val = self_->dot(v);
        return return_val;
    }

    double PxExtendedVec3_distanceSquared(physx_PxExtendedVec3 const* self__pod, physx_PxExtendedVec3 const* v_pod) {
        physx::PxExtendedVec3 const* self_ = reinterpret_cast<physx::PxExtendedVec3 const*>(self__pod);
        physx::PxExtendedVec3 const& v = reinterpret_cast<physx::PxExtendedVec3 const&>(*v_pod);
        double return_val = self_->distanceSquared(v);
        return return_val;
    }

    double PxExtendedVec3_magnitudeSquared(physx_PxExtendedVec3 const* self__pod) {
        physx::PxExtendedVec3 const* self_ = reinterpret_cast<physx::PxExtendedVec3 const*>(self__pod);
        double return_val = self_->magnitudeSquared();
        return return_val;
    }

    double PxExtendedVec3_magnitude(physx_PxExtendedVec3 const* self__pod) {
        physx::PxExtendedVec3 const* self_ = reinterpret_cast<physx::PxExtendedVec3 const*>(self__pod);
        double return_val = self_->magnitude();
        return return_val;
    }

    double PxExtendedVec3_normalize(physx_PxExtendedVec3* self__pod) {
        physx::PxExtendedVec3* self_ = reinterpret_cast<physx::PxExtendedVec3*>(self__pod);
        double return_val = self_->normalize();
        return return_val;
    }

    bool PxExtendedVec3_isFinite(physx_PxExtendedVec3 const* self__pod) {
        physx::PxExtendedVec3 const* self_ = reinterpret_cast<physx::PxExtendedVec3 const*>(self__pod);
        bool return_val = self_->isFinite();
        return return_val;
    }

    void PxExtendedVec3_maximum(physx_PxExtendedVec3* self__pod, physx_PxExtendedVec3 const* v_pod) {
        physx::PxExtendedVec3* self_ = reinterpret_cast<physx::PxExtendedVec3*>(self__pod);
        physx::PxExtendedVec3 const& v = reinterpret_cast<physx::PxExtendedVec3 const&>(*v_pod);
        self_->maximum(v);
    }

    void PxExtendedVec3_minimum(physx_PxExtendedVec3* self__pod, physx_PxExtendedVec3 const* v_pod) {
        physx::PxExtendedVec3* self_ = reinterpret_cast<physx::PxExtendedVec3*>(self__pod);
        physx::PxExtendedVec3 const& v = reinterpret_cast<physx::PxExtendedVec3 const&>(*v_pod);
        self_->minimum(v);
    }

    void PxExtendedVec3_set(physx_PxExtendedVec3* self__pod, double x_, double y_, double z_) {
        physx::PxExtendedVec3* self_ = reinterpret_cast<physx::PxExtendedVec3*>(self__pod);
        self_->set(x_, y_, z_);
    }

    void PxExtendedVec3_setPlusInfinity(physx_PxExtendedVec3* self__pod) {
        physx::PxExtendedVec3* self_ = reinterpret_cast<physx::PxExtendedVec3*>(self__pod);
        self_->setPlusInfinity();
    }

    void PxExtendedVec3_setMinusInfinity(physx_PxExtendedVec3* self__pod) {
        physx::PxExtendedVec3* self_ = reinterpret_cast<physx::PxExtendedVec3*>(self__pod);
        self_->setMinusInfinity();
    }

    void PxExtendedVec3_cross(physx_PxExtendedVec3* self__pod, physx_PxExtendedVec3 const* left_pod, physx_PxVec3 const* right_pod) {
        physx::PxExtendedVec3* self_ = reinterpret_cast<physx::PxExtendedVec3*>(self__pod);
        physx::PxExtendedVec3 const& left = reinterpret_cast<physx::PxExtendedVec3 const&>(*left_pod);
        physx::PxVec3 const& right = reinterpret_cast<physx::PxVec3 const&>(*right_pod);
        self_->cross(left, right);
    }

    void PxExtendedVec3_cross_1(physx_PxExtendedVec3* self__pod, physx_PxExtendedVec3 const* left_pod, physx_PxExtendedVec3 const* right_pod) {
        physx::PxExtendedVec3* self_ = reinterpret_cast<physx::PxExtendedVec3*>(self__pod);
        physx::PxExtendedVec3 const& left = reinterpret_cast<physx::PxExtendedVec3 const&>(*left_pod);
        physx::PxExtendedVec3 const& right = reinterpret_cast<physx::PxExtendedVec3 const&>(*right_pod);
        self_->cross(left, right);
    }

    physx_PxExtendedVec3 PxExtendedVec3_cross_2(physx_PxExtendedVec3 const* self__pod, physx_PxExtendedVec3 const* v_pod) {
        physx::PxExtendedVec3 const* self_ = reinterpret_cast<physx::PxExtendedVec3 const*>(self__pod);
        physx::PxExtendedVec3 const& v = reinterpret_cast<physx::PxExtendedVec3 const&>(*v_pod);
        physx::PxExtendedVec3 return_val = self_->cross(v);
        physx_PxExtendedVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxExtendedVec3_cross_3(physx_PxExtendedVec3* self__pod, physx_PxVec3 const* left_pod, physx_PxExtendedVec3 const* right_pod) {
        physx::PxExtendedVec3* self_ = reinterpret_cast<physx::PxExtendedVec3*>(self__pod);
        physx::PxVec3 const& left = reinterpret_cast<physx::PxVec3 const&>(*left_pod);
        physx::PxExtendedVec3 const& right = reinterpret_cast<physx::PxExtendedVec3 const&>(*right_pod);
        self_->cross(left, right);
    }

    physx_PxVec3 phys_toVec3(physx_PxExtendedVec3 const* v_pod) {
        physx::PxExtendedVec3 const& v = reinterpret_cast<physx::PxExtendedVec3 const&>(*v_pod);
        physx::PxVec3 return_val = toVec3(v);
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    PxGeometryType PxObstacle_getType(physx_PxObstacle const* self__pod) {
        physx::PxObstacle const* self_ = reinterpret_cast<physx::PxObstacle const*>(self__pod);
        physx::PxGeometryType::Enum return_val = self_->getType();
        PxGeometryType return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxBoxObstacle PxBoxObstacle_new() {
        PxBoxObstacle return_val;
        physx_PxBoxObstacle return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxCapsuleObstacle PxCapsuleObstacle_new() {
        PxCapsuleObstacle return_val;
        physx_PxCapsuleObstacle return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxObstacleContext_release(physx_PxObstacleContext* self__pod) {
        physx::PxObstacleContext* self_ = reinterpret_cast<physx::PxObstacleContext*>(self__pod);
        self_->release();
    }

    physx_PxControllerManager* PxObstacleContext_getControllerManager(physx_PxObstacleContext const* self__pod) {
        physx::PxObstacleContext const* self_ = reinterpret_cast<physx::PxObstacleContext const*>(self__pod);
        physx::PxControllerManager& return_val = self_->getControllerManager();
        auto return_val_pod = reinterpret_cast<physx_PxControllerManager*>(&return_val);
        return return_val_pod;
    }

    uint32_t PxObstacleContext_addObstacle(physx_PxObstacleContext* self__pod, physx_PxObstacle const* obstacle_pod) {
        physx::PxObstacleContext* self_ = reinterpret_cast<physx::PxObstacleContext*>(self__pod);
        physx::PxObstacle const& obstacle = reinterpret_cast<physx::PxObstacle const&>(*obstacle_pod);
        uint32_t return_val = self_->addObstacle(obstacle);
        return return_val;
    }

    bool PxObstacleContext_removeObstacle(physx_PxObstacleContext* self__pod, uint32_t handle) {
        physx::PxObstacleContext* self_ = reinterpret_cast<physx::PxObstacleContext*>(self__pod);
        bool return_val = self_->removeObstacle(handle);
        return return_val;
    }

    bool PxObstacleContext_updateObstacle(physx_PxObstacleContext* self__pod, uint32_t handle, physx_PxObstacle const* obstacle_pod) {
        physx::PxObstacleContext* self_ = reinterpret_cast<physx::PxObstacleContext*>(self__pod);
        physx::PxObstacle const& obstacle = reinterpret_cast<physx::PxObstacle const&>(*obstacle_pod);
        bool return_val = self_->updateObstacle(handle, obstacle);
        return return_val;
    }

    uint32_t PxObstacleContext_getNbObstacles(physx_PxObstacleContext const* self__pod) {
        physx::PxObstacleContext const* self_ = reinterpret_cast<physx::PxObstacleContext const*>(self__pod);
        uint32_t return_val = self_->getNbObstacles();
        return return_val;
    }

    physx_PxObstacle const* PxObstacleContext_getObstacle(physx_PxObstacleContext const* self__pod, uint32_t i) {
        physx::PxObstacleContext const* self_ = reinterpret_cast<physx::PxObstacleContext const*>(self__pod);
        physx::PxObstacle const* return_val = self_->getObstacle(i);
        auto return_val_pod = reinterpret_cast<physx_PxObstacle const*>(return_val);
        return return_val_pod;
    }

    physx_PxObstacle const* PxObstacleContext_getObstacleByHandle(physx_PxObstacleContext const* self__pod, uint32_t handle) {
        physx::PxObstacleContext const* self_ = reinterpret_cast<physx::PxObstacleContext const*>(self__pod);
        physx::PxObstacle const* return_val = self_->getObstacleByHandle(handle);
        auto return_val_pod = reinterpret_cast<physx_PxObstacle const*>(return_val);
        return return_val_pod;
    }

    void PxUserControllerHitReport_onShapeHit(physx_PxUserControllerHitReport* self__pod, physx_PxControllerShapeHit const* hit_pod) {
        physx::PxUserControllerHitReport* self_ = reinterpret_cast<physx::PxUserControllerHitReport*>(self__pod);
        physx::PxControllerShapeHit const& hit = reinterpret_cast<physx::PxControllerShapeHit const&>(*hit_pod);
        self_->onShapeHit(hit);
    }

    void PxUserControllerHitReport_onControllerHit(physx_PxUserControllerHitReport* self__pod, physx_PxControllersHit const* hit_pod) {
        physx::PxUserControllerHitReport* self_ = reinterpret_cast<physx::PxUserControllerHitReport*>(self__pod);
        physx::PxControllersHit const& hit = reinterpret_cast<physx::PxControllersHit const&>(*hit_pod);
        self_->onControllerHit(hit);
    }

    void PxUserControllerHitReport_onObstacleHit(physx_PxUserControllerHitReport* self__pod, physx_PxControllerObstacleHit const* hit_pod) {
        physx::PxUserControllerHitReport* self_ = reinterpret_cast<physx::PxUserControllerHitReport*>(self__pod);
        physx::PxControllerObstacleHit const& hit = reinterpret_cast<physx::PxControllerObstacleHit const&>(*hit_pod);
        self_->onObstacleHit(hit);
    }

    void PxControllerFilterCallback_delete(physx_PxControllerFilterCallback* self__pod) {
        physx::PxControllerFilterCallback* self_ = reinterpret_cast<physx::PxControllerFilterCallback*>(self__pod);
        delete self_;
    }

    bool PxControllerFilterCallback_filter(physx_PxControllerFilterCallback* self__pod, physx_PxController const* a_pod, physx_PxController const* b_pod) {
        physx::PxControllerFilterCallback* self_ = reinterpret_cast<physx::PxControllerFilterCallback*>(self__pod);
        physx::PxController const& a = reinterpret_cast<physx::PxController const&>(*a_pod);
        physx::PxController const& b = reinterpret_cast<physx::PxController const&>(*b_pod);
        bool return_val = self_->filter(a, b);
        return return_val;
    }

    physx_PxControllerFilters PxControllerFilters_new(physx_PxFilterData const* filterData_pod, physx_PxQueryFilterCallback* cb_pod, physx_PxControllerFilterCallback* cctFilterCb_pod) {
        physx::PxFilterData const* filterData = reinterpret_cast<physx::PxFilterData const*>(filterData_pod);
        physx::PxQueryFilterCallback* cb = reinterpret_cast<physx::PxQueryFilterCallback*>(cb_pod);
        physx::PxControllerFilterCallback* cctFilterCb = reinterpret_cast<physx::PxControllerFilterCallback*>(cctFilterCb_pod);
        PxControllerFilters return_val(filterData, cb, cctFilterCb);
        physx_PxControllerFilters return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxControllerDesc_isValid(physx_PxControllerDesc const* self__pod) {
        physx::PxControllerDesc const* self_ = reinterpret_cast<physx::PxControllerDesc const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    PxControllerShapeType PxControllerDesc_getType(physx_PxControllerDesc const* self__pod) {
        physx::PxControllerDesc const* self_ = reinterpret_cast<physx::PxControllerDesc const*>(self__pod);
        physx::PxControllerShapeType::Enum return_val = self_->getType();
        PxControllerShapeType return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    PxControllerShapeType PxController_getType(physx_PxController const* self__pod) {
        physx::PxController const* self_ = reinterpret_cast<physx::PxController const*>(self__pod);
        physx::PxControllerShapeType::Enum return_val = self_->getType();
        PxControllerShapeType return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxController_release(physx_PxController* self__pod) {
        physx::PxController* self_ = reinterpret_cast<physx::PxController*>(self__pod);
        self_->release();
    }

    PxControllerCollisionFlags PxController_move(physx_PxController* self__pod, physx_PxVec3 const* disp_pod, float minDist, float elapsedTime, physx_PxControllerFilters const* filters_pod, physx_PxObstacleContext const* obstacles_pod) {
        physx::PxController* self_ = reinterpret_cast<physx::PxController*>(self__pod);
        physx::PxVec3 const& disp = reinterpret_cast<physx::PxVec3 const&>(*disp_pod);
        physx::PxControllerFilters const& filters = reinterpret_cast<physx::PxControllerFilters const&>(*filters_pod);
        physx::PxObstacleContext const* obstacles = reinterpret_cast<physx::PxObstacleContext const*>(obstacles_pod);
        physx::PxControllerCollisionFlags return_val = self_->move(disp, minDist, elapsedTime, filters, obstacles);
        PxControllerCollisionFlags return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxController_setPosition(physx_PxController* self__pod, physx_PxExtendedVec3 const* position_pod) {
        physx::PxController* self_ = reinterpret_cast<physx::PxController*>(self__pod);
        physx::PxExtendedVec3 const& position = reinterpret_cast<physx::PxExtendedVec3 const&>(*position_pod);
        bool return_val = self_->setPosition(position);
        return return_val;
    }

    physx_PxExtendedVec3 const* PxController_getPosition(physx_PxController const* self__pod) {
        physx::PxController const* self_ = reinterpret_cast<physx::PxController const*>(self__pod);
        physx::PxExtendedVec3 const& return_val = self_->getPosition();
        auto return_val_pod = reinterpret_cast<physx_PxExtendedVec3 const*>(&return_val);
        return return_val_pod;
    }

    bool PxController_setFootPosition(physx_PxController* self__pod, physx_PxExtendedVec3 const* position_pod) {
        physx::PxController* self_ = reinterpret_cast<physx::PxController*>(self__pod);
        physx::PxExtendedVec3 const& position = reinterpret_cast<physx::PxExtendedVec3 const&>(*position_pod);
        bool return_val = self_->setFootPosition(position);
        return return_val;
    }

    physx_PxExtendedVec3 PxController_getFootPosition(physx_PxController const* self__pod) {
        physx::PxController const* self_ = reinterpret_cast<physx::PxController const*>(self__pod);
        physx::PxExtendedVec3 return_val = self_->getFootPosition();
        physx_PxExtendedVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxRigidDynamic* PxController_getActor(physx_PxController const* self__pod) {
        physx::PxController const* self_ = reinterpret_cast<physx::PxController const*>(self__pod);
        physx::PxRigidDynamic* return_val = self_->getActor();
        auto return_val_pod = reinterpret_cast<physx_PxRigidDynamic*>(return_val);
        return return_val_pod;
    }

    void PxController_setStepOffset(physx_PxController* self__pod, float offset) {
        physx::PxController* self_ = reinterpret_cast<physx::PxController*>(self__pod);
        self_->setStepOffset(offset);
    }

    float PxController_getStepOffset(physx_PxController const* self__pod) {
        physx::PxController const* self_ = reinterpret_cast<physx::PxController const*>(self__pod);
        float return_val = self_->getStepOffset();
        return return_val;
    }

    void PxController_setNonWalkableMode(physx_PxController* self__pod, PxControllerNonWalkableMode flag_pod) {
        physx::PxController* self_ = reinterpret_cast<physx::PxController*>(self__pod);
        auto flag = static_cast<physx::PxControllerNonWalkableMode::Enum>(flag_pod);
        self_->setNonWalkableMode(flag);
    }

    PxControllerNonWalkableMode PxController_getNonWalkableMode(physx_PxController const* self__pod) {
        physx::PxController const* self_ = reinterpret_cast<physx::PxController const*>(self__pod);
        physx::PxControllerNonWalkableMode::Enum return_val = self_->getNonWalkableMode();
        PxControllerNonWalkableMode return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    float PxController_getContactOffset(physx_PxController const* self__pod) {
        physx::PxController const* self_ = reinterpret_cast<physx::PxController const*>(self__pod);
        float return_val = self_->getContactOffset();
        return return_val;
    }

    void PxController_setContactOffset(physx_PxController* self__pod, float offset) {
        physx::PxController* self_ = reinterpret_cast<physx::PxController*>(self__pod);
        self_->setContactOffset(offset);
    }

    physx_PxVec3 PxController_getUpDirection(physx_PxController const* self__pod) {
        physx::PxController const* self_ = reinterpret_cast<physx::PxController const*>(self__pod);
        physx::PxVec3 return_val = self_->getUpDirection();
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxController_setUpDirection(physx_PxController* self__pod, physx_PxVec3 const* up_pod) {
        physx::PxController* self_ = reinterpret_cast<physx::PxController*>(self__pod);
        physx::PxVec3 const& up = reinterpret_cast<physx::PxVec3 const&>(*up_pod);
        self_->setUpDirection(up);
    }

    float PxController_getSlopeLimit(physx_PxController const* self__pod) {
        physx::PxController const* self_ = reinterpret_cast<physx::PxController const*>(self__pod);
        float return_val = self_->getSlopeLimit();
        return return_val;
    }

    void PxController_setSlopeLimit(physx_PxController* self__pod, float slopeLimit) {
        physx::PxController* self_ = reinterpret_cast<physx::PxController*>(self__pod);
        self_->setSlopeLimit(slopeLimit);
    }

    void PxController_invalidateCache(physx_PxController* self__pod) {
        physx::PxController* self_ = reinterpret_cast<physx::PxController*>(self__pod);
        self_->invalidateCache();
    }

    physx_PxScene* PxController_getScene(physx_PxController* self__pod) {
        physx::PxController* self_ = reinterpret_cast<physx::PxController*>(self__pod);
        physx::PxScene* return_val = self_->getScene();
        auto return_val_pod = reinterpret_cast<physx_PxScene*>(return_val);
        return return_val_pod;
    }

    void* PxController_getUserData(physx_PxController const* self__pod) {
        physx::PxController const* self_ = reinterpret_cast<physx::PxController const*>(self__pod);
        void* return_val = self_->getUserData();
        return return_val;
    }

    void PxController_setUserData(physx_PxController* self__pod, void* userData) {
        physx::PxController* self_ = reinterpret_cast<physx::PxController*>(self__pod);
        self_->setUserData(userData);
    }

    void PxController_getState(physx_PxController const* self__pod, physx_PxControllerState* state_pod) {
        physx::PxController const* self_ = reinterpret_cast<physx::PxController const*>(self__pod);
        physx::PxControllerState& state = reinterpret_cast<physx::PxControllerState&>(*state_pod);
        self_->getState(state);
    }

    void PxController_getStats(physx_PxController const* self__pod, physx_PxControllerStats* stats_pod) {
        physx::PxController const* self_ = reinterpret_cast<physx::PxController const*>(self__pod);
        physx::PxControllerStats& stats = reinterpret_cast<physx::PxControllerStats&>(*stats_pod);
        self_->getStats(stats);
    }

    void PxController_resize(physx_PxController* self__pod, float height) {
        physx::PxController* self_ = reinterpret_cast<physx::PxController*>(self__pod);
        self_->resize(height);
    }

    physx_PxBoxControllerDesc* PxBoxControllerDesc_new_alloc() {
        auto return_val = new physx::PxBoxControllerDesc();
        auto return_val_pod = reinterpret_cast<physx_PxBoxControllerDesc*>(return_val);
        return return_val_pod;
    }

    void PxBoxControllerDesc_delete(physx_PxBoxControllerDesc* self__pod) {
        physx::PxBoxControllerDesc* self_ = reinterpret_cast<physx::PxBoxControllerDesc*>(self__pod);
        delete self_;
    }

    void PxBoxControllerDesc_setToDefault(physx_PxBoxControllerDesc* self__pod) {
        physx::PxBoxControllerDesc* self_ = reinterpret_cast<physx::PxBoxControllerDesc*>(self__pod);
        self_->setToDefault();
    }

    bool PxBoxControllerDesc_isValid(physx_PxBoxControllerDesc const* self__pod) {
        physx::PxBoxControllerDesc const* self_ = reinterpret_cast<physx::PxBoxControllerDesc const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    float PxBoxController_getHalfHeight(physx_PxBoxController const* self__pod) {
        physx::PxBoxController const* self_ = reinterpret_cast<physx::PxBoxController const*>(self__pod);
        float return_val = self_->getHalfHeight();
        return return_val;
    }

    float PxBoxController_getHalfSideExtent(physx_PxBoxController const* self__pod) {
        physx::PxBoxController const* self_ = reinterpret_cast<physx::PxBoxController const*>(self__pod);
        float return_val = self_->getHalfSideExtent();
        return return_val;
    }

    float PxBoxController_getHalfForwardExtent(physx_PxBoxController const* self__pod) {
        physx::PxBoxController const* self_ = reinterpret_cast<physx::PxBoxController const*>(self__pod);
        float return_val = self_->getHalfForwardExtent();
        return return_val;
    }

    bool PxBoxController_setHalfHeight(physx_PxBoxController* self__pod, float halfHeight) {
        physx::PxBoxController* self_ = reinterpret_cast<physx::PxBoxController*>(self__pod);
        bool return_val = self_->setHalfHeight(halfHeight);
        return return_val;
    }

    bool PxBoxController_setHalfSideExtent(physx_PxBoxController* self__pod, float halfSideExtent) {
        physx::PxBoxController* self_ = reinterpret_cast<physx::PxBoxController*>(self__pod);
        bool return_val = self_->setHalfSideExtent(halfSideExtent);
        return return_val;
    }

    bool PxBoxController_setHalfForwardExtent(physx_PxBoxController* self__pod, float halfForwardExtent) {
        physx::PxBoxController* self_ = reinterpret_cast<physx::PxBoxController*>(self__pod);
        bool return_val = self_->setHalfForwardExtent(halfForwardExtent);
        return return_val;
    }

    physx_PxCapsuleControllerDesc* PxCapsuleControllerDesc_new_alloc() {
        auto return_val = new physx::PxCapsuleControllerDesc();
        auto return_val_pod = reinterpret_cast<physx_PxCapsuleControllerDesc*>(return_val);
        return return_val_pod;
    }

    void PxCapsuleControllerDesc_delete(physx_PxCapsuleControllerDesc* self__pod) {
        physx::PxCapsuleControllerDesc* self_ = reinterpret_cast<physx::PxCapsuleControllerDesc*>(self__pod);
        delete self_;
    }

    void PxCapsuleControllerDesc_setToDefault(physx_PxCapsuleControllerDesc* self__pod) {
        physx::PxCapsuleControllerDesc* self_ = reinterpret_cast<physx::PxCapsuleControllerDesc*>(self__pod);
        self_->setToDefault();
    }

    bool PxCapsuleControllerDesc_isValid(physx_PxCapsuleControllerDesc const* self__pod) {
        physx::PxCapsuleControllerDesc const* self_ = reinterpret_cast<physx::PxCapsuleControllerDesc const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    float PxCapsuleController_getRadius(physx_PxCapsuleController const* self__pod) {
        physx::PxCapsuleController const* self_ = reinterpret_cast<physx::PxCapsuleController const*>(self__pod);
        float return_val = self_->getRadius();
        return return_val;
    }

    bool PxCapsuleController_setRadius(physx_PxCapsuleController* self__pod, float radius) {
        physx::PxCapsuleController* self_ = reinterpret_cast<physx::PxCapsuleController*>(self__pod);
        bool return_val = self_->setRadius(radius);
        return return_val;
    }

    float PxCapsuleController_getHeight(physx_PxCapsuleController const* self__pod) {
        physx::PxCapsuleController const* self_ = reinterpret_cast<physx::PxCapsuleController const*>(self__pod);
        float return_val = self_->getHeight();
        return return_val;
    }

    bool PxCapsuleController_setHeight(physx_PxCapsuleController* self__pod, float height) {
        physx::PxCapsuleController* self_ = reinterpret_cast<physx::PxCapsuleController*>(self__pod);
        bool return_val = self_->setHeight(height);
        return return_val;
    }

    PxCapsuleClimbingMode PxCapsuleController_getClimbingMode(physx_PxCapsuleController const* self__pod) {
        physx::PxCapsuleController const* self_ = reinterpret_cast<physx::PxCapsuleController const*>(self__pod);
        physx::PxCapsuleClimbingMode::Enum return_val = self_->getClimbingMode();
        PxCapsuleClimbingMode return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxCapsuleController_setClimbingMode(physx_PxCapsuleController* self__pod, PxCapsuleClimbingMode mode_pod) {
        physx::PxCapsuleController* self_ = reinterpret_cast<physx::PxCapsuleController*>(self__pod);
        auto mode = static_cast<physx::PxCapsuleClimbingMode::Enum>(mode_pod);
        bool return_val = self_->setClimbingMode(mode);
        return return_val;
    }

    PxControllerBehaviorFlags PxControllerBehaviorCallback_getBehaviorFlags(physx_PxControllerBehaviorCallback* self__pod, physx_PxShape const* shape_pod, physx_PxActor const* actor_pod) {
        physx::PxControllerBehaviorCallback* self_ = reinterpret_cast<physx::PxControllerBehaviorCallback*>(self__pod);
        physx::PxShape const& shape = reinterpret_cast<physx::PxShape const&>(*shape_pod);
        physx::PxActor const& actor = reinterpret_cast<physx::PxActor const&>(*actor_pod);
        physx::PxControllerBehaviorFlags return_val = self_->getBehaviorFlags(shape, actor);
        PxControllerBehaviorFlags return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    PxControllerBehaviorFlags PxControllerBehaviorCallback_getBehaviorFlags_1(physx_PxControllerBehaviorCallback* self__pod, physx_PxController const* controller_pod) {
        physx::PxControllerBehaviorCallback* self_ = reinterpret_cast<physx::PxControllerBehaviorCallback*>(self__pod);
        physx::PxController const& controller = reinterpret_cast<physx::PxController const&>(*controller_pod);
        physx::PxControllerBehaviorFlags return_val = self_->getBehaviorFlags(controller);
        PxControllerBehaviorFlags return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    PxControllerBehaviorFlags PxControllerBehaviorCallback_getBehaviorFlags_2(physx_PxControllerBehaviorCallback* self__pod, physx_PxObstacle const* obstacle_pod) {
        physx::PxControllerBehaviorCallback* self_ = reinterpret_cast<physx::PxControllerBehaviorCallback*>(self__pod);
        physx::PxObstacle const& obstacle = reinterpret_cast<physx::PxObstacle const&>(*obstacle_pod);
        physx::PxControllerBehaviorFlags return_val = self_->getBehaviorFlags(obstacle);
        PxControllerBehaviorFlags return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxControllerManager_release(physx_PxControllerManager* self__pod) {
        physx::PxControllerManager* self_ = reinterpret_cast<physx::PxControllerManager*>(self__pod);
        self_->release();
    }

    physx_PxScene* PxControllerManager_getScene(physx_PxControllerManager const* self__pod) {
        physx::PxControllerManager const* self_ = reinterpret_cast<physx::PxControllerManager const*>(self__pod);
        physx::PxScene& return_val = self_->getScene();
        auto return_val_pod = reinterpret_cast<physx_PxScene*>(&return_val);
        return return_val_pod;
    }

    uint32_t PxControllerManager_getNbControllers(physx_PxControllerManager const* self__pod) {
        physx::PxControllerManager const* self_ = reinterpret_cast<physx::PxControllerManager const*>(self__pod);
        uint32_t return_val = self_->getNbControllers();
        return return_val;
    }

    physx_PxController* PxControllerManager_getController(physx_PxControllerManager* self__pod, uint32_t index) {
        physx::PxControllerManager* self_ = reinterpret_cast<physx::PxControllerManager*>(self__pod);
        physx::PxController* return_val = self_->getController(index);
        auto return_val_pod = reinterpret_cast<physx_PxController*>(return_val);
        return return_val_pod;
    }

    physx_PxController* PxControllerManager_createController(physx_PxControllerManager* self__pod, physx_PxControllerDesc const* desc_pod) {
        physx::PxControllerManager* self_ = reinterpret_cast<physx::PxControllerManager*>(self__pod);
        physx::PxControllerDesc const& desc = reinterpret_cast<physx::PxControllerDesc const&>(*desc_pod);
        physx::PxController* return_val = self_->createController(desc);
        auto return_val_pod = reinterpret_cast<physx_PxController*>(return_val);
        return return_val_pod;
    }

    void PxControllerManager_purgeControllers(physx_PxControllerManager* self__pod) {
        physx::PxControllerManager* self_ = reinterpret_cast<physx::PxControllerManager*>(self__pod);
        self_->purgeControllers();
    }

    physx_PxRenderBuffer* PxControllerManager_getRenderBuffer(physx_PxControllerManager* self__pod) {
        physx::PxControllerManager* self_ = reinterpret_cast<physx::PxControllerManager*>(self__pod);
        physx::PxRenderBuffer& return_val = self_->getRenderBuffer();
        auto return_val_pod = reinterpret_cast<physx_PxRenderBuffer*>(&return_val);
        return return_val_pod;
    }

    void PxControllerManager_setDebugRenderingFlags(physx_PxControllerManager* self__pod, PxControllerDebugRenderFlags flags_pod) {
        physx::PxControllerManager* self_ = reinterpret_cast<physx::PxControllerManager*>(self__pod);
        auto flags = physx::PxControllerDebugRenderFlags(flags_pod);
        self_->setDebugRenderingFlags(flags);
    }

    uint32_t PxControllerManager_getNbObstacleContexts(physx_PxControllerManager const* self__pod) {
        physx::PxControllerManager const* self_ = reinterpret_cast<physx::PxControllerManager const*>(self__pod);
        uint32_t return_val = self_->getNbObstacleContexts();
        return return_val;
    }

    physx_PxObstacleContext* PxControllerManager_getObstacleContext(physx_PxControllerManager* self__pod, uint32_t index) {
        physx::PxControllerManager* self_ = reinterpret_cast<physx::PxControllerManager*>(self__pod);
        physx::PxObstacleContext* return_val = self_->getObstacleContext(index);
        auto return_val_pod = reinterpret_cast<physx_PxObstacleContext*>(return_val);
        return return_val_pod;
    }

    physx_PxObstacleContext* PxControllerManager_createObstacleContext(physx_PxControllerManager* self__pod) {
        physx::PxControllerManager* self_ = reinterpret_cast<physx::PxControllerManager*>(self__pod);
        physx::PxObstacleContext* return_val = self_->createObstacleContext();
        auto return_val_pod = reinterpret_cast<physx_PxObstacleContext*>(return_val);
        return return_val_pod;
    }

    void PxControllerManager_computeInteractions(physx_PxControllerManager* self__pod, float elapsedTime, physx_PxControllerFilterCallback* cctFilterCb_pod) {
        physx::PxControllerManager* self_ = reinterpret_cast<physx::PxControllerManager*>(self__pod);
        physx::PxControllerFilterCallback* cctFilterCb = reinterpret_cast<physx::PxControllerFilterCallback*>(cctFilterCb_pod);
        self_->computeInteractions(elapsedTime, cctFilterCb);
    }

    void PxControllerManager_setTessellation(physx_PxControllerManager* self__pod, bool flag, float maxEdgeLength) {
        physx::PxControllerManager* self_ = reinterpret_cast<physx::PxControllerManager*>(self__pod);
        self_->setTessellation(flag, maxEdgeLength);
    }

    void PxControllerManager_setOverlapRecoveryModule(physx_PxControllerManager* self__pod, bool flag) {
        physx::PxControllerManager* self_ = reinterpret_cast<physx::PxControllerManager*>(self__pod);
        self_->setOverlapRecoveryModule(flag);
    }

    void PxControllerManager_setPreciseSweeps(physx_PxControllerManager* self__pod, bool flag) {
        physx::PxControllerManager* self_ = reinterpret_cast<physx::PxControllerManager*>(self__pod);
        self_->setPreciseSweeps(flag);
    }

    void PxControllerManager_setPreventVerticalSlidingAgainstCeiling(physx_PxControllerManager* self__pod, bool flag) {
        physx::PxControllerManager* self_ = reinterpret_cast<physx::PxControllerManager*>(self__pod);
        self_->setPreventVerticalSlidingAgainstCeiling(flag);
    }

    void PxControllerManager_shiftOrigin(physx_PxControllerManager* self__pod, physx_PxVec3 const* shift_pod) {
        physx::PxControllerManager* self_ = reinterpret_cast<physx::PxControllerManager*>(self__pod);
        physx::PxVec3 const& shift = reinterpret_cast<physx::PxVec3 const&>(*shift_pod);
        self_->shiftOrigin(shift);
    }

    physx_PxControllerManager* phys_PxCreateControllerManager(physx_PxScene* scene_pod, bool lockingEnabled) {
        physx::PxScene& scene = reinterpret_cast<physx::PxScene&>(*scene_pod);
        physx::PxControllerManager* return_val = PxCreateControllerManager(scene, lockingEnabled);
        auto return_val_pod = reinterpret_cast<physx_PxControllerManager*>(return_val);
        return return_val_pod;
    }

    physx_PxDim3 PxDim3_new() {
        PxDim3 return_val;
        physx_PxDim3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxSDFDesc PxSDFDesc_new() {
        PxSDFDesc return_val;
        physx_PxSDFDesc return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxSDFDesc_isValid(physx_PxSDFDesc const* self__pod) {
        physx::PxSDFDesc const* self_ = reinterpret_cast<physx::PxSDFDesc const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    physx_PxConvexMeshDesc PxConvexMeshDesc_new() {
        PxConvexMeshDesc return_val;
        physx_PxConvexMeshDesc return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxConvexMeshDesc_setToDefault(physx_PxConvexMeshDesc* self__pod) {
        physx::PxConvexMeshDesc* self_ = reinterpret_cast<physx::PxConvexMeshDesc*>(self__pod);
        self_->setToDefault();
    }

    bool PxConvexMeshDesc_isValid(physx_PxConvexMeshDesc const* self__pod) {
        physx::PxConvexMeshDesc const* self_ = reinterpret_cast<physx::PxConvexMeshDesc const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    physx_PxTriangleMeshDesc PxTriangleMeshDesc_new() {
        PxTriangleMeshDesc return_val;
        physx_PxTriangleMeshDesc return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxTriangleMeshDesc_setToDefault(physx_PxTriangleMeshDesc* self__pod) {
        physx::PxTriangleMeshDesc* self_ = reinterpret_cast<physx::PxTriangleMeshDesc*>(self__pod);
        self_->setToDefault();
    }

    bool PxTriangleMeshDesc_isValid(physx_PxTriangleMeshDesc const* self__pod) {
        physx::PxTriangleMeshDesc const* self_ = reinterpret_cast<physx::PxTriangleMeshDesc const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    physx_PxTetrahedronMeshDesc PxTetrahedronMeshDesc_new() {
        PxTetrahedronMeshDesc return_val;
        physx_PxTetrahedronMeshDesc return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxTetrahedronMeshDesc_isValid(physx_PxTetrahedronMeshDesc const* self__pod) {
        physx::PxTetrahedronMeshDesc const* self_ = reinterpret_cast<physx::PxTetrahedronMeshDesc const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    physx_PxSoftBodySimulationDataDesc PxSoftBodySimulationDataDesc_new() {
        PxSoftBodySimulationDataDesc return_val;
        physx_PxSoftBodySimulationDataDesc return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxSoftBodySimulationDataDesc_isValid(physx_PxSoftBodySimulationDataDesc const* self__pod) {
        physx::PxSoftBodySimulationDataDesc const* self_ = reinterpret_cast<physx::PxSoftBodySimulationDataDesc const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    void PxBVH34MidphaseDesc_setToDefault(physx_PxBVH34MidphaseDesc* self__pod) {
        physx::PxBVH34MidphaseDesc* self_ = reinterpret_cast<physx::PxBVH34MidphaseDesc*>(self__pod);
        self_->setToDefault();
    }

    bool PxBVH34MidphaseDesc_isValid(physx_PxBVH34MidphaseDesc const* self__pod) {
        physx::PxBVH34MidphaseDesc const* self_ = reinterpret_cast<physx::PxBVH34MidphaseDesc const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    physx_PxMidphaseDesc PxMidphaseDesc_new() {
        PxMidphaseDesc return_val;
        physx_PxMidphaseDesc return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    PxMeshMidPhase PxMidphaseDesc_getType(physx_PxMidphaseDesc const* self__pod) {
        physx::PxMidphaseDesc const* self_ = reinterpret_cast<physx::PxMidphaseDesc const*>(self__pod);
        physx::PxMeshMidPhase::Enum return_val = self_->getType();
        PxMeshMidPhase return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxMidphaseDesc_setToDefault(physx_PxMidphaseDesc* self__pod, PxMeshMidPhase type_pod) {
        physx::PxMidphaseDesc* self_ = reinterpret_cast<physx::PxMidphaseDesc*>(self__pod);
        auto type = static_cast<physx::PxMeshMidPhase::Enum>(type_pod);
        self_->setToDefault(type);
    }

    bool PxMidphaseDesc_isValid(physx_PxMidphaseDesc const* self__pod) {
        physx::PxMidphaseDesc const* self_ = reinterpret_cast<physx::PxMidphaseDesc const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    physx_PxBVHDesc PxBVHDesc_new() {
        PxBVHDesc return_val;
        physx_PxBVHDesc return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxBVHDesc_setToDefault(physx_PxBVHDesc* self__pod) {
        physx::PxBVHDesc* self_ = reinterpret_cast<physx::PxBVHDesc*>(self__pod);
        self_->setToDefault();
    }

    bool PxBVHDesc_isValid(physx_PxBVHDesc const* self__pod) {
        physx::PxBVHDesc const* self_ = reinterpret_cast<physx::PxBVHDesc const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    physx_PxCookingParams PxCookingParams_new(physx_PxTolerancesScale const* sc_pod) {
        physx::PxTolerancesScale const& sc = reinterpret_cast<physx::PxTolerancesScale const&>(*sc_pod);
        PxCookingParams return_val(sc);
        physx_PxCookingParams return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxInsertionCallback* phys_PxGetStandaloneInsertionCallback() {
        physx::PxInsertionCallback* return_val = PxGetStandaloneInsertionCallback();
        auto return_val_pod = reinterpret_cast<physx_PxInsertionCallback*>(return_val);
        return return_val_pod;
    }

    bool phys_PxCookBVH(physx_PxBVHDesc const* desc_pod, physx_PxOutputStream* stream_pod) {
        physx::PxBVHDesc const& desc = reinterpret_cast<physx::PxBVHDesc const&>(*desc_pod);
        physx::PxOutputStream& stream = reinterpret_cast<physx::PxOutputStream&>(*stream_pod);
        bool return_val = PxCookBVH(desc, stream);
        return return_val;
    }

    physx_PxBVH* phys_PxCreateBVH(physx_PxBVHDesc const* desc_pod, physx_PxInsertionCallback* insertionCallback_pod) {
        physx::PxBVHDesc const& desc = reinterpret_cast<physx::PxBVHDesc const&>(*desc_pod);
        physx::PxInsertionCallback& insertionCallback = reinterpret_cast<physx::PxInsertionCallback&>(*insertionCallback_pod);
        physx::PxBVH* return_val = PxCreateBVH(desc, insertionCallback);
        auto return_val_pod = reinterpret_cast<physx_PxBVH*>(return_val);
        return return_val_pod;
    }

    bool phys_PxCookHeightField(physx_PxHeightFieldDesc const* desc_pod, physx_PxOutputStream* stream_pod) {
        physx::PxHeightFieldDesc const& desc = reinterpret_cast<physx::PxHeightFieldDesc const&>(*desc_pod);
        physx::PxOutputStream& stream = reinterpret_cast<physx::PxOutputStream&>(*stream_pod);
        bool return_val = PxCookHeightField(desc, stream);
        return return_val;
    }

    physx_PxHeightField* phys_PxCreateHeightField(physx_PxHeightFieldDesc const* desc_pod, physx_PxInsertionCallback* insertionCallback_pod) {
        physx::PxHeightFieldDesc const& desc = reinterpret_cast<physx::PxHeightFieldDesc const&>(*desc_pod);
        physx::PxInsertionCallback& insertionCallback = reinterpret_cast<physx::PxInsertionCallback&>(*insertionCallback_pod);
        physx::PxHeightField* return_val = PxCreateHeightField(desc, insertionCallback);
        auto return_val_pod = reinterpret_cast<physx_PxHeightField*>(return_val);
        return return_val_pod;
    }

    bool phys_PxCookConvexMesh(physx_PxCookingParams const* params_pod, physx_PxConvexMeshDesc const* desc_pod, physx_PxOutputStream* stream_pod, PxConvexMeshCookingResult* condition_pod) {
        physx::PxCookingParams const& params = reinterpret_cast<physx::PxCookingParams const&>(*params_pod);
        physx::PxConvexMeshDesc const& desc = reinterpret_cast<physx::PxConvexMeshDesc const&>(*desc_pod);
        physx::PxOutputStream& stream = reinterpret_cast<physx::PxOutputStream&>(*stream_pod);
        physx::PxConvexMeshCookingResult::Enum* condition = reinterpret_cast<physx::PxConvexMeshCookingResult::Enum*>(condition_pod);
        bool return_val = PxCookConvexMesh(params, desc, stream, condition);
        return return_val;
    }

    physx_PxConvexMesh* phys_PxCreateConvexMesh(physx_PxCookingParams const* params_pod, physx_PxConvexMeshDesc const* desc_pod, physx_PxInsertionCallback* insertionCallback_pod, PxConvexMeshCookingResult* condition_pod) {
        physx::PxCookingParams const& params = reinterpret_cast<physx::PxCookingParams const&>(*params_pod);
        physx::PxConvexMeshDesc const& desc = reinterpret_cast<physx::PxConvexMeshDesc const&>(*desc_pod);
        physx::PxInsertionCallback& insertionCallback = reinterpret_cast<physx::PxInsertionCallback&>(*insertionCallback_pod);
        physx::PxConvexMeshCookingResult::Enum* condition = reinterpret_cast<physx::PxConvexMeshCookingResult::Enum*>(condition_pod);
        physx::PxConvexMesh* return_val = PxCreateConvexMesh(params, desc, insertionCallback, condition);
        auto return_val_pod = reinterpret_cast<physx_PxConvexMesh*>(return_val);
        return return_val_pod;
    }

    bool phys_PxValidateConvexMesh(physx_PxCookingParams const* params_pod, physx_PxConvexMeshDesc const* desc_pod) {
        physx::PxCookingParams const& params = reinterpret_cast<physx::PxCookingParams const&>(*params_pod);
        physx::PxConvexMeshDesc const& desc = reinterpret_cast<physx::PxConvexMeshDesc const&>(*desc_pod);
        bool return_val = PxValidateConvexMesh(params, desc);
        return return_val;
    }

    bool phys_PxComputeHullPolygons(physx_PxCookingParams const* params_pod, physx_PxSimpleTriangleMesh const* mesh_pod, physx_PxAllocatorCallback* inCallback_pod, uint32_t* nbVerts_pod, physx_PxVec3** vertices_pod, uint32_t* nbIndices_pod, uint32_t** indices_pod, uint32_t* nbPolygons_pod, physx_PxHullPolygon** hullPolygons_pod) {
        physx::PxCookingParams const& params = reinterpret_cast<physx::PxCookingParams const&>(*params_pod);
        physx::PxSimpleTriangleMesh const& mesh = reinterpret_cast<physx::PxSimpleTriangleMesh const&>(*mesh_pod);
        physx::PxAllocatorCallback& inCallback = reinterpret_cast<physx::PxAllocatorCallback&>(*inCallback_pod);
        uint32_t& nbVerts = *nbVerts_pod;
        physx::PxVec3*& vertices = reinterpret_cast<physx::PxVec3*&>(*vertices_pod);
        uint32_t& nbIndices = *nbIndices_pod;
        uint32_t*& indices = reinterpret_cast<uint32_t*&>(*indices_pod);
        uint32_t& nbPolygons = *nbPolygons_pod;
        physx::PxHullPolygon*& hullPolygons = reinterpret_cast<physx::PxHullPolygon*&>(*hullPolygons_pod);
        bool return_val = PxComputeHullPolygons(params, mesh, inCallback, nbVerts, vertices, nbIndices, indices, nbPolygons, hullPolygons);
        return return_val;
    }

    bool phys_PxValidateTriangleMesh(physx_PxCookingParams const* params_pod, physx_PxTriangleMeshDesc const* desc_pod) {
        physx::PxCookingParams const& params = reinterpret_cast<physx::PxCookingParams const&>(*params_pod);
        physx::PxTriangleMeshDesc const& desc = reinterpret_cast<physx::PxTriangleMeshDesc const&>(*desc_pod);
        bool return_val = PxValidateTriangleMesh(params, desc);
        return return_val;
    }

    physx_PxTriangleMesh* phys_PxCreateTriangleMesh(physx_PxCookingParams const* params_pod, physx_PxTriangleMeshDesc const* desc_pod, physx_PxInsertionCallback* insertionCallback_pod, PxTriangleMeshCookingResult* condition_pod) {
        physx::PxCookingParams const& params = reinterpret_cast<physx::PxCookingParams const&>(*params_pod);
        physx::PxTriangleMeshDesc const& desc = reinterpret_cast<physx::PxTriangleMeshDesc const&>(*desc_pod);
        physx::PxInsertionCallback& insertionCallback = reinterpret_cast<physx::PxInsertionCallback&>(*insertionCallback_pod);
        physx::PxTriangleMeshCookingResult::Enum* condition = reinterpret_cast<physx::PxTriangleMeshCookingResult::Enum*>(condition_pod);
        physx::PxTriangleMesh* return_val = PxCreateTriangleMesh(params, desc, insertionCallback, condition);
        auto return_val_pod = reinterpret_cast<physx_PxTriangleMesh*>(return_val);
        return return_val_pod;
    }

    bool phys_PxCookTriangleMesh(physx_PxCookingParams const* params_pod, physx_PxTriangleMeshDesc const* desc_pod, physx_PxOutputStream* stream_pod, PxTriangleMeshCookingResult* condition_pod) {
        physx::PxCookingParams const& params = reinterpret_cast<physx::PxCookingParams const&>(*params_pod);
        physx::PxTriangleMeshDesc const& desc = reinterpret_cast<physx::PxTriangleMeshDesc const&>(*desc_pod);
        physx::PxOutputStream& stream = reinterpret_cast<physx::PxOutputStream&>(*stream_pod);
        physx::PxTriangleMeshCookingResult::Enum* condition = reinterpret_cast<physx::PxTriangleMeshCookingResult::Enum*>(condition_pod);
        bool return_val = PxCookTriangleMesh(params, desc, stream, condition);
        return return_val;
    }

    physx_PxDefaultMemoryOutputStream* PxDefaultMemoryOutputStream_new_alloc(physx_PxAllocatorCallback* allocator_pod) {
        physx::PxAllocatorCallback& allocator = reinterpret_cast<physx::PxAllocatorCallback&>(*allocator_pod);
        auto return_val = new physx::PxDefaultMemoryOutputStream(allocator);
        auto return_val_pod = reinterpret_cast<physx_PxDefaultMemoryOutputStream*>(return_val);
        return return_val_pod;
    }

    void PxDefaultMemoryOutputStream_delete(physx_PxDefaultMemoryOutputStream* self__pod) {
        physx::PxDefaultMemoryOutputStream* self_ = reinterpret_cast<physx::PxDefaultMemoryOutputStream*>(self__pod);
        delete self_;
    }

    uint32_t PxDefaultMemoryOutputStream_write(physx_PxDefaultMemoryOutputStream* self__pod, void const* src, uint32_t count) {
        physx::PxDefaultMemoryOutputStream* self_ = reinterpret_cast<physx::PxDefaultMemoryOutputStream*>(self__pod);
        uint32_t return_val = self_->write(src, count);
        return return_val;
    }

    uint32_t PxDefaultMemoryOutputStream_getSize(physx_PxDefaultMemoryOutputStream const* self__pod) {
        physx::PxDefaultMemoryOutputStream const* self_ = reinterpret_cast<physx::PxDefaultMemoryOutputStream const*>(self__pod);
        uint32_t return_val = self_->getSize();
        return return_val;
    }

    uint8_t* PxDefaultMemoryOutputStream_getData(physx_PxDefaultMemoryOutputStream const* self__pod) {
        physx::PxDefaultMemoryOutputStream const* self_ = reinterpret_cast<physx::PxDefaultMemoryOutputStream const*>(self__pod);
        uint8_t* return_val = self_->getData();
        return return_val;
    }

    physx_PxDefaultMemoryInputData* PxDefaultMemoryInputData_new_alloc(uint8_t* data, uint32_t length) {
        auto return_val = new physx::PxDefaultMemoryInputData(data, length);
        auto return_val_pod = reinterpret_cast<physx_PxDefaultMemoryInputData*>(return_val);
        return return_val_pod;
    }

    uint32_t PxDefaultMemoryInputData_read(physx_PxDefaultMemoryInputData* self__pod, void* dest, uint32_t count) {
        physx::PxDefaultMemoryInputData* self_ = reinterpret_cast<physx::PxDefaultMemoryInputData*>(self__pod);
        uint32_t return_val = self_->read(dest, count);
        return return_val;
    }

    uint32_t PxDefaultMemoryInputData_getLength(physx_PxDefaultMemoryInputData const* self__pod) {
        physx::PxDefaultMemoryInputData const* self_ = reinterpret_cast<physx::PxDefaultMemoryInputData const*>(self__pod);
        uint32_t return_val = self_->getLength();
        return return_val;
    }

    void PxDefaultMemoryInputData_seek(physx_PxDefaultMemoryInputData* self__pod, uint32_t pos) {
        physx::PxDefaultMemoryInputData* self_ = reinterpret_cast<physx::PxDefaultMemoryInputData*>(self__pod);
        self_->seek(pos);
    }

    uint32_t PxDefaultMemoryInputData_tell(physx_PxDefaultMemoryInputData const* self__pod) {
        physx::PxDefaultMemoryInputData const* self_ = reinterpret_cast<physx::PxDefaultMemoryInputData const*>(self__pod);
        uint32_t return_val = self_->tell();
        return return_val;
    }

    physx_PxDefaultFileOutputStream* PxDefaultFileOutputStream_new_alloc(char const* name) {
        auto return_val = new physx::PxDefaultFileOutputStream(name);
        auto return_val_pod = reinterpret_cast<physx_PxDefaultFileOutputStream*>(return_val);
        return return_val_pod;
    }

    void PxDefaultFileOutputStream_delete(physx_PxDefaultFileOutputStream* self__pod) {
        physx::PxDefaultFileOutputStream* self_ = reinterpret_cast<physx::PxDefaultFileOutputStream*>(self__pod);
        delete self_;
    }

    uint32_t PxDefaultFileOutputStream_write(physx_PxDefaultFileOutputStream* self__pod, void const* src, uint32_t count) {
        physx::PxDefaultFileOutputStream* self_ = reinterpret_cast<physx::PxDefaultFileOutputStream*>(self__pod);
        uint32_t return_val = self_->write(src, count);
        return return_val;
    }

    bool PxDefaultFileOutputStream_isValid(physx_PxDefaultFileOutputStream* self__pod) {
        physx::PxDefaultFileOutputStream* self_ = reinterpret_cast<physx::PxDefaultFileOutputStream*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    physx_PxDefaultFileInputData* PxDefaultFileInputData_new_alloc(char const* name) {
        auto return_val = new physx::PxDefaultFileInputData(name);
        auto return_val_pod = reinterpret_cast<physx_PxDefaultFileInputData*>(return_val);
        return return_val_pod;
    }

    void PxDefaultFileInputData_delete(physx_PxDefaultFileInputData* self__pod) {
        physx::PxDefaultFileInputData* self_ = reinterpret_cast<physx::PxDefaultFileInputData*>(self__pod);
        delete self_;
    }

    uint32_t PxDefaultFileInputData_read(physx_PxDefaultFileInputData* self__pod, void* dest, uint32_t count) {
        physx::PxDefaultFileInputData* self_ = reinterpret_cast<physx::PxDefaultFileInputData*>(self__pod);
        uint32_t return_val = self_->read(dest, count);
        return return_val;
    }

    void PxDefaultFileInputData_seek(physx_PxDefaultFileInputData* self__pod, uint32_t pos) {
        physx::PxDefaultFileInputData* self_ = reinterpret_cast<physx::PxDefaultFileInputData*>(self__pod);
        self_->seek(pos);
    }

    uint32_t PxDefaultFileInputData_tell(physx_PxDefaultFileInputData const* self__pod) {
        physx::PxDefaultFileInputData const* self_ = reinterpret_cast<physx::PxDefaultFileInputData const*>(self__pod);
        uint32_t return_val = self_->tell();
        return return_val;
    }

    uint32_t PxDefaultFileInputData_getLength(physx_PxDefaultFileInputData const* self__pod) {
        physx::PxDefaultFileInputData const* self_ = reinterpret_cast<physx::PxDefaultFileInputData const*>(self__pod);
        uint32_t return_val = self_->getLength();
        return return_val;
    }

    bool PxDefaultFileInputData_isValid(physx_PxDefaultFileInputData const* self__pod) {
        physx::PxDefaultFileInputData const* self_ = reinterpret_cast<physx::PxDefaultFileInputData const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    void* phys_platformAlignedAlloc(size_t size_pod) {
        size_t size;
        memcpy(&size, &size_pod, sizeof(size));
        void* return_val = platformAlignedAlloc(size);
        return return_val;
    }

    void phys_platformAlignedFree(void* ptr) {
        platformAlignedFree(ptr);
    }

    void* PxDefaultAllocator_allocate(physx_PxDefaultAllocator* self__pod, size_t size_pod, char const* anon_param1, char const* anon_param2, int32_t anon_param3) {
        physx::PxDefaultAllocator* self_ = reinterpret_cast<physx::PxDefaultAllocator*>(self__pod);
        size_t size;
        memcpy(&size, &size_pod, sizeof(size));
        void* return_val = self_->allocate(size, anon_param1, anon_param2, anon_param3);
        return return_val;
    }

    void PxDefaultAllocator_deallocate(physx_PxDefaultAllocator* self__pod, void* ptr) {
        physx::PxDefaultAllocator* self_ = reinterpret_cast<physx::PxDefaultAllocator*>(self__pod);
        self_->deallocate(ptr);
    }

    void PxDefaultAllocator_delete(physx_PxDefaultAllocator* self__pod) {
        physx::PxDefaultAllocator* self_ = reinterpret_cast<physx::PxDefaultAllocator*>(self__pod);
        delete self_;
    }

    void PxJoint_setActors(physx_PxJoint* self__pod, physx_PxRigidActor* actor0_pod, physx_PxRigidActor* actor1_pod) {
        physx::PxJoint* self_ = reinterpret_cast<physx::PxJoint*>(self__pod);
        physx::PxRigidActor* actor0 = reinterpret_cast<physx::PxRigidActor*>(actor0_pod);
        physx::PxRigidActor* actor1 = reinterpret_cast<physx::PxRigidActor*>(actor1_pod);
        self_->setActors(actor0, actor1);
    }

    void PxJoint_getActors(physx_PxJoint const* self__pod, physx_PxRigidActor** actor0_pod, physx_PxRigidActor** actor1_pod) {
        physx::PxJoint const* self_ = reinterpret_cast<physx::PxJoint const*>(self__pod);
        physx::PxRigidActor*& actor0 = reinterpret_cast<physx::PxRigidActor*&>(*actor0_pod);
        physx::PxRigidActor*& actor1 = reinterpret_cast<physx::PxRigidActor*&>(*actor1_pod);
        self_->getActors(actor0, actor1);
    }

    void PxJoint_setLocalPose(physx_PxJoint* self__pod, PxJointActorIndex actor_pod, physx_PxTransform const* localPose_pod) {
        physx::PxJoint* self_ = reinterpret_cast<physx::PxJoint*>(self__pod);
        auto actor = static_cast<physx::PxJointActorIndex::Enum>(actor_pod);
        physx::PxTransform const& localPose = reinterpret_cast<physx::PxTransform const&>(*localPose_pod);
        self_->setLocalPose(actor, localPose);
    }

    physx_PxTransform PxJoint_getLocalPose(physx_PxJoint const* self__pod, PxJointActorIndex actor_pod) {
        physx::PxJoint const* self_ = reinterpret_cast<physx::PxJoint const*>(self__pod);
        auto actor = static_cast<physx::PxJointActorIndex::Enum>(actor_pod);
        physx::PxTransform return_val = self_->getLocalPose(actor);
        physx_PxTransform return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxTransform PxJoint_getRelativeTransform(physx_PxJoint const* self__pod) {
        physx::PxJoint const* self_ = reinterpret_cast<physx::PxJoint const*>(self__pod);
        physx::PxTransform return_val = self_->getRelativeTransform();
        physx_PxTransform return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3 PxJoint_getRelativeLinearVelocity(physx_PxJoint const* self__pod) {
        physx::PxJoint const* self_ = reinterpret_cast<physx::PxJoint const*>(self__pod);
        physx::PxVec3 return_val = self_->getRelativeLinearVelocity();
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3 PxJoint_getRelativeAngularVelocity(physx_PxJoint const* self__pod) {
        physx::PxJoint const* self_ = reinterpret_cast<physx::PxJoint const*>(self__pod);
        physx::PxVec3 return_val = self_->getRelativeAngularVelocity();
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxJoint_setBreakForce(physx_PxJoint* self__pod, float force, float torque) {
        physx::PxJoint* self_ = reinterpret_cast<physx::PxJoint*>(self__pod);
        self_->setBreakForce(force, torque);
    }

    void PxJoint_getBreakForce(physx_PxJoint const* self__pod, float* force_pod, float* torque_pod) {
        physx::PxJoint const* self_ = reinterpret_cast<physx::PxJoint const*>(self__pod);
        float& force = *force_pod;
        float& torque = *torque_pod;
        self_->getBreakForce(force, torque);
    }

    void PxJoint_setConstraintFlags(physx_PxJoint* self__pod, PxConstraintFlags flags_pod) {
        physx::PxJoint* self_ = reinterpret_cast<physx::PxJoint*>(self__pod);
        auto flags = physx::PxConstraintFlags(flags_pod);
        self_->setConstraintFlags(flags);
    }

    void PxJoint_setConstraintFlag(physx_PxJoint* self__pod, PxConstraintFlag flag_pod, bool value) {
        physx::PxJoint* self_ = reinterpret_cast<physx::PxJoint*>(self__pod);
        auto flag = static_cast<physx::PxConstraintFlag::Enum>(flag_pod);
        self_->setConstraintFlag(flag, value);
    }

    PxConstraintFlags PxJoint_getConstraintFlags(physx_PxJoint const* self__pod) {
        physx::PxJoint const* self_ = reinterpret_cast<physx::PxJoint const*>(self__pod);
        physx::PxConstraintFlags return_val = self_->getConstraintFlags();
        PxConstraintFlags return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxJoint_setInvMassScale0(physx_PxJoint* self__pod, float invMassScale) {
        physx::PxJoint* self_ = reinterpret_cast<physx::PxJoint*>(self__pod);
        self_->setInvMassScale0(invMassScale);
    }

    float PxJoint_getInvMassScale0(physx_PxJoint const* self__pod) {
        physx::PxJoint const* self_ = reinterpret_cast<physx::PxJoint const*>(self__pod);
        float return_val = self_->getInvMassScale0();
        return return_val;
    }

    void PxJoint_setInvInertiaScale0(physx_PxJoint* self__pod, float invInertiaScale) {
        physx::PxJoint* self_ = reinterpret_cast<physx::PxJoint*>(self__pod);
        self_->setInvInertiaScale0(invInertiaScale);
    }

    float PxJoint_getInvInertiaScale0(physx_PxJoint const* self__pod) {
        physx::PxJoint const* self_ = reinterpret_cast<physx::PxJoint const*>(self__pod);
        float return_val = self_->getInvInertiaScale0();
        return return_val;
    }

    void PxJoint_setInvMassScale1(physx_PxJoint* self__pod, float invMassScale) {
        physx::PxJoint* self_ = reinterpret_cast<physx::PxJoint*>(self__pod);
        self_->setInvMassScale1(invMassScale);
    }

    float PxJoint_getInvMassScale1(physx_PxJoint const* self__pod) {
        physx::PxJoint const* self_ = reinterpret_cast<physx::PxJoint const*>(self__pod);
        float return_val = self_->getInvMassScale1();
        return return_val;
    }

    void PxJoint_setInvInertiaScale1(physx_PxJoint* self__pod, float invInertiaScale) {
        physx::PxJoint* self_ = reinterpret_cast<physx::PxJoint*>(self__pod);
        self_->setInvInertiaScale1(invInertiaScale);
    }

    float PxJoint_getInvInertiaScale1(physx_PxJoint const* self__pod) {
        physx::PxJoint const* self_ = reinterpret_cast<physx::PxJoint const*>(self__pod);
        float return_val = self_->getInvInertiaScale1();
        return return_val;
    }

    physx_PxConstraint* PxJoint_getConstraint(physx_PxJoint const* self__pod) {
        physx::PxJoint const* self_ = reinterpret_cast<physx::PxJoint const*>(self__pod);
        physx::PxConstraint* return_val = self_->getConstraint();
        auto return_val_pod = reinterpret_cast<physx_PxConstraint*>(return_val);
        return return_val_pod;
    }

    void PxJoint_setName(physx_PxJoint* self__pod, char const* name) {
        physx::PxJoint* self_ = reinterpret_cast<physx::PxJoint*>(self__pod);
        self_->setName(name);
    }

    char const* PxJoint_getName(physx_PxJoint const* self__pod) {
        physx::PxJoint const* self_ = reinterpret_cast<physx::PxJoint const*>(self__pod);
        char const* return_val = self_->getName();
        return return_val;
    }

    void PxJoint_release(physx_PxJoint* self__pod) {
        physx::PxJoint* self_ = reinterpret_cast<physx::PxJoint*>(self__pod);
        self_->release();
    }

    physx_PxScene* PxJoint_getScene(physx_PxJoint const* self__pod) {
        physx::PxJoint const* self_ = reinterpret_cast<physx::PxJoint const*>(self__pod);
        physx::PxScene* return_val = self_->getScene();
        auto return_val_pod = reinterpret_cast<physx_PxScene*>(return_val);
        return return_val_pod;
    }

    void PxJoint_getBinaryMetaData(physx_PxOutputStream* stream_pod) {
        physx::PxOutputStream& stream = reinterpret_cast<physx::PxOutputStream&>(*stream_pod);
        PxJoint::getBinaryMetaData(stream);
    }

    physx_PxSpring PxSpring_new(float stiffness_, float damping_) {
        PxSpring return_val(stiffness_, damping_);
        physx_PxSpring return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void phys_PxSetJointGlobalFrame(physx_PxJoint* joint_pod, physx_PxVec3 const* wsAnchor_pod, physx_PxVec3 const* wsAxis_pod) {
        physx::PxJoint& joint = reinterpret_cast<physx::PxJoint&>(*joint_pod);
        physx::PxVec3 const* wsAnchor = reinterpret_cast<physx::PxVec3 const*>(wsAnchor_pod);
        physx::PxVec3 const* wsAxis = reinterpret_cast<physx::PxVec3 const*>(wsAxis_pod);
        PxSetJointGlobalFrame(joint, wsAnchor, wsAxis);
    }

    physx_PxDistanceJoint* phys_PxDistanceJointCreate(physx_PxPhysics* physics_pod, physx_PxRigidActor* actor0_pod, physx_PxTransform const* localFrame0_pod, physx_PxRigidActor* actor1_pod, physx_PxTransform const* localFrame1_pod) {
        physx::PxPhysics& physics = reinterpret_cast<physx::PxPhysics&>(*physics_pod);
        physx::PxRigidActor* actor0 = reinterpret_cast<physx::PxRigidActor*>(actor0_pod);
        physx::PxTransform const& localFrame0 = reinterpret_cast<physx::PxTransform const&>(*localFrame0_pod);
        physx::PxRigidActor* actor1 = reinterpret_cast<physx::PxRigidActor*>(actor1_pod);
        physx::PxTransform const& localFrame1 = reinterpret_cast<physx::PxTransform const&>(*localFrame1_pod);
        physx::PxDistanceJoint* return_val = PxDistanceJointCreate(physics, actor0, localFrame0, actor1, localFrame1);
        auto return_val_pod = reinterpret_cast<physx_PxDistanceJoint*>(return_val);
        return return_val_pod;
    }

    float PxDistanceJoint_getDistance(physx_PxDistanceJoint const* self__pod) {
        physx::PxDistanceJoint const* self_ = reinterpret_cast<physx::PxDistanceJoint const*>(self__pod);
        float return_val = self_->getDistance();
        return return_val;
    }

    void PxDistanceJoint_setMinDistance(physx_PxDistanceJoint* self__pod, float distance) {
        physx::PxDistanceJoint* self_ = reinterpret_cast<physx::PxDistanceJoint*>(self__pod);
        self_->setMinDistance(distance);
    }

    float PxDistanceJoint_getMinDistance(physx_PxDistanceJoint const* self__pod) {
        physx::PxDistanceJoint const* self_ = reinterpret_cast<physx::PxDistanceJoint const*>(self__pod);
        float return_val = self_->getMinDistance();
        return return_val;
    }

    void PxDistanceJoint_setMaxDistance(physx_PxDistanceJoint* self__pod, float distance) {
        physx::PxDistanceJoint* self_ = reinterpret_cast<physx::PxDistanceJoint*>(self__pod);
        self_->setMaxDistance(distance);
    }

    float PxDistanceJoint_getMaxDistance(physx_PxDistanceJoint const* self__pod) {
        physx::PxDistanceJoint const* self_ = reinterpret_cast<physx::PxDistanceJoint const*>(self__pod);
        float return_val = self_->getMaxDistance();
        return return_val;
    }

    void PxDistanceJoint_setTolerance(physx_PxDistanceJoint* self__pod, float tolerance) {
        physx::PxDistanceJoint* self_ = reinterpret_cast<physx::PxDistanceJoint*>(self__pod);
        self_->setTolerance(tolerance);
    }

    float PxDistanceJoint_getTolerance(physx_PxDistanceJoint const* self__pod) {
        physx::PxDistanceJoint const* self_ = reinterpret_cast<physx::PxDistanceJoint const*>(self__pod);
        float return_val = self_->getTolerance();
        return return_val;
    }

    void PxDistanceJoint_setStiffness(physx_PxDistanceJoint* self__pod, float stiffness) {
        physx::PxDistanceJoint* self_ = reinterpret_cast<physx::PxDistanceJoint*>(self__pod);
        self_->setStiffness(stiffness);
    }

    float PxDistanceJoint_getStiffness(physx_PxDistanceJoint const* self__pod) {
        physx::PxDistanceJoint const* self_ = reinterpret_cast<physx::PxDistanceJoint const*>(self__pod);
        float return_val = self_->getStiffness();
        return return_val;
    }

    void PxDistanceJoint_setDamping(physx_PxDistanceJoint* self__pod, float damping) {
        physx::PxDistanceJoint* self_ = reinterpret_cast<physx::PxDistanceJoint*>(self__pod);
        self_->setDamping(damping);
    }

    float PxDistanceJoint_getDamping(physx_PxDistanceJoint const* self__pod) {
        physx::PxDistanceJoint const* self_ = reinterpret_cast<physx::PxDistanceJoint const*>(self__pod);
        float return_val = self_->getDamping();
        return return_val;
    }

    void PxDistanceJoint_setContactDistance(physx_PxDistanceJoint* self__pod, float contactDistance) {
        physx::PxDistanceJoint* self_ = reinterpret_cast<physx::PxDistanceJoint*>(self__pod);
        self_->setContactDistance(contactDistance);
    }

    float PxDistanceJoint_getContactDistance(physx_PxDistanceJoint const* self__pod) {
        physx::PxDistanceJoint const* self_ = reinterpret_cast<physx::PxDistanceJoint const*>(self__pod);
        float return_val = self_->getContactDistance();
        return return_val;
    }

    void PxDistanceJoint_setDistanceJointFlags(physx_PxDistanceJoint* self__pod, PxDistanceJointFlags flags_pod) {
        physx::PxDistanceJoint* self_ = reinterpret_cast<physx::PxDistanceJoint*>(self__pod);
        auto flags = physx::PxDistanceJointFlags(flags_pod);
        self_->setDistanceJointFlags(flags);
    }

    void PxDistanceJoint_setDistanceJointFlag(physx_PxDistanceJoint* self__pod, PxDistanceJointFlag flag_pod, bool value) {
        physx::PxDistanceJoint* self_ = reinterpret_cast<physx::PxDistanceJoint*>(self__pod);
        auto flag = static_cast<physx::PxDistanceJointFlag::Enum>(flag_pod);
        self_->setDistanceJointFlag(flag, value);
    }

    PxDistanceJointFlags PxDistanceJoint_getDistanceJointFlags(physx_PxDistanceJoint const* self__pod) {
        physx::PxDistanceJoint const* self_ = reinterpret_cast<physx::PxDistanceJoint const*>(self__pod);
        physx::PxDistanceJointFlags return_val = self_->getDistanceJointFlags();
        PxDistanceJointFlags return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    char const* PxDistanceJoint_getConcreteTypeName(physx_PxDistanceJoint const* self__pod) {
        physx::PxDistanceJoint const* self_ = reinterpret_cast<physx::PxDistanceJoint const*>(self__pod);
        char const* return_val = self_->getConcreteTypeName();
        return return_val;
    }

    physx_PxContactJoint* phys_PxContactJointCreate(physx_PxPhysics* physics_pod, physx_PxRigidActor* actor0_pod, physx_PxTransform const* localFrame0_pod, physx_PxRigidActor* actor1_pod, physx_PxTransform const* localFrame1_pod) {
        physx::PxPhysics& physics = reinterpret_cast<physx::PxPhysics&>(*physics_pod);
        physx::PxRigidActor* actor0 = reinterpret_cast<physx::PxRigidActor*>(actor0_pod);
        physx::PxTransform const& localFrame0 = reinterpret_cast<physx::PxTransform const&>(*localFrame0_pod);
        physx::PxRigidActor* actor1 = reinterpret_cast<physx::PxRigidActor*>(actor1_pod);
        physx::PxTransform const& localFrame1 = reinterpret_cast<physx::PxTransform const&>(*localFrame1_pod);
        physx::PxContactJoint* return_val = PxContactJointCreate(physics, actor0, localFrame0, actor1, localFrame1);
        auto return_val_pod = reinterpret_cast<physx_PxContactJoint*>(return_val);
        return return_val_pod;
    }

    physx_PxJacobianRow PxJacobianRow_new() {
        PxJacobianRow return_val;
        physx_PxJacobianRow return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxJacobianRow PxJacobianRow_new_1(physx_PxVec3 const* lin0_pod, physx_PxVec3 const* lin1_pod, physx_PxVec3 const* ang0_pod, physx_PxVec3 const* ang1_pod) {
        physx::PxVec3 const& lin0 = reinterpret_cast<physx::PxVec3 const&>(*lin0_pod);
        physx::PxVec3 const& lin1 = reinterpret_cast<physx::PxVec3 const&>(*lin1_pod);
        physx::PxVec3 const& ang0 = reinterpret_cast<physx::PxVec3 const&>(*ang0_pod);
        physx::PxVec3 const& ang1 = reinterpret_cast<physx::PxVec3 const&>(*ang1_pod);
        PxJacobianRow return_val(lin0, lin1, ang0, ang1);
        physx_PxJacobianRow return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxContactJoint_setContact(physx_PxContactJoint* self__pod, physx_PxVec3 const* contact_pod) {
        physx::PxContactJoint* self_ = reinterpret_cast<physx::PxContactJoint*>(self__pod);
        physx::PxVec3 const& contact = reinterpret_cast<physx::PxVec3 const&>(*contact_pod);
        self_->setContact(contact);
    }

    void PxContactJoint_setContactNormal(physx_PxContactJoint* self__pod, physx_PxVec3 const* contactNormal_pod) {
        physx::PxContactJoint* self_ = reinterpret_cast<physx::PxContactJoint*>(self__pod);
        physx::PxVec3 const& contactNormal = reinterpret_cast<physx::PxVec3 const&>(*contactNormal_pod);
        self_->setContactNormal(contactNormal);
    }

    void PxContactJoint_setPenetration(physx_PxContactJoint* self__pod, float penetration) {
        physx::PxContactJoint* self_ = reinterpret_cast<physx::PxContactJoint*>(self__pod);
        self_->setPenetration(penetration);
    }

    physx_PxVec3 PxContactJoint_getContact(physx_PxContactJoint const* self__pod) {
        physx::PxContactJoint const* self_ = reinterpret_cast<physx::PxContactJoint const*>(self__pod);
        physx::PxVec3 return_val = self_->getContact();
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3 PxContactJoint_getContactNormal(physx_PxContactJoint const* self__pod) {
        physx::PxContactJoint const* self_ = reinterpret_cast<physx::PxContactJoint const*>(self__pod);
        physx::PxVec3 return_val = self_->getContactNormal();
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    float PxContactJoint_getPenetration(physx_PxContactJoint const* self__pod) {
        physx::PxContactJoint const* self_ = reinterpret_cast<physx::PxContactJoint const*>(self__pod);
        float return_val = self_->getPenetration();
        return return_val;
    }

    float PxContactJoint_getRestitution(physx_PxContactJoint const* self__pod) {
        physx::PxContactJoint const* self_ = reinterpret_cast<physx::PxContactJoint const*>(self__pod);
        float return_val = self_->getRestitution();
        return return_val;
    }

    void PxContactJoint_setRestitution(physx_PxContactJoint* self__pod, float restitution) {
        physx::PxContactJoint* self_ = reinterpret_cast<physx::PxContactJoint*>(self__pod);
        self_->setRestitution(restitution);
    }

    float PxContactJoint_getBounceThreshold(physx_PxContactJoint const* self__pod) {
        physx::PxContactJoint const* self_ = reinterpret_cast<physx::PxContactJoint const*>(self__pod);
        float return_val = self_->getBounceThreshold();
        return return_val;
    }

    void PxContactJoint_setBounceThreshold(physx_PxContactJoint* self__pod, float bounceThreshold) {
        physx::PxContactJoint* self_ = reinterpret_cast<physx::PxContactJoint*>(self__pod);
        self_->setBounceThreshold(bounceThreshold);
    }

    char const* PxContactJoint_getConcreteTypeName(physx_PxContactJoint const* self__pod) {
        physx::PxContactJoint const* self_ = reinterpret_cast<physx::PxContactJoint const*>(self__pod);
        char const* return_val = self_->getConcreteTypeName();
        return return_val;
    }

    void PxContactJoint_computeJacobians(physx_PxContactJoint const* self__pod, physx_PxJacobianRow* jacobian_pod) {
        physx::PxContactJoint const* self_ = reinterpret_cast<physx::PxContactJoint const*>(self__pod);
        physx::PxJacobianRow* jacobian = reinterpret_cast<physx::PxJacobianRow*>(jacobian_pod);
        self_->computeJacobians(jacobian);
    }

    uint32_t PxContactJoint_getNbJacobianRows(physx_PxContactJoint const* self__pod) {
        physx::PxContactJoint const* self_ = reinterpret_cast<physx::PxContactJoint const*>(self__pod);
        uint32_t return_val = self_->getNbJacobianRows();
        return return_val;
    }

    physx_PxFixedJoint* phys_PxFixedJointCreate(physx_PxPhysics* physics_pod, physx_PxRigidActor* actor0_pod, physx_PxTransform const* localFrame0_pod, physx_PxRigidActor* actor1_pod, physx_PxTransform const* localFrame1_pod) {
        physx::PxPhysics& physics = reinterpret_cast<physx::PxPhysics&>(*physics_pod);
        physx::PxRigidActor* actor0 = reinterpret_cast<physx::PxRigidActor*>(actor0_pod);
        physx::PxTransform const& localFrame0 = reinterpret_cast<physx::PxTransform const&>(*localFrame0_pod);
        physx::PxRigidActor* actor1 = reinterpret_cast<physx::PxRigidActor*>(actor1_pod);
        physx::PxTransform const& localFrame1 = reinterpret_cast<physx::PxTransform const&>(*localFrame1_pod);
        physx::PxFixedJoint* return_val = PxFixedJointCreate(physics, actor0, localFrame0, actor1, localFrame1);
        auto return_val_pod = reinterpret_cast<physx_PxFixedJoint*>(return_val);
        return return_val_pod;
    }

    char const* PxFixedJoint_getConcreteTypeName(physx_PxFixedJoint const* self__pod) {
        physx::PxFixedJoint const* self_ = reinterpret_cast<physx::PxFixedJoint const*>(self__pod);
        char const* return_val = self_->getConcreteTypeName();
        return return_val;
    }

    physx_PxJointLimitParameters* PxJointLimitParameters_new_alloc() {
        auto return_val = new physx::PxJointLimitParameters();
        auto return_val_pod = reinterpret_cast<physx_PxJointLimitParameters*>(return_val);
        return return_val_pod;
    }

    bool PxJointLimitParameters_isValid(physx_PxJointLimitParameters const* self__pod) {
        physx::PxJointLimitParameters const* self_ = reinterpret_cast<physx::PxJointLimitParameters const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    bool PxJointLimitParameters_isSoft(physx_PxJointLimitParameters const* self__pod) {
        physx::PxJointLimitParameters const* self_ = reinterpret_cast<physx::PxJointLimitParameters const*>(self__pod);
        bool return_val = self_->isSoft();
        return return_val;
    }

    physx_PxJointLinearLimit PxJointLinearLimit_new(physx_PxTolerancesScale const* scale_pod, float extent, float contactDist_deprecated) {
        physx::PxTolerancesScale const& scale = reinterpret_cast<physx::PxTolerancesScale const&>(*scale_pod);
        PxJointLinearLimit return_val(scale, extent, contactDist_deprecated);
        physx_PxJointLinearLimit return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxJointLinearLimit PxJointLinearLimit_new_1(float extent, physx_PxSpring const* spring_pod) {
        physx::PxSpring const& spring = reinterpret_cast<physx::PxSpring const&>(*spring_pod);
        PxJointLinearLimit return_val(extent, spring);
        physx_PxJointLinearLimit return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxJointLinearLimit_isValid(physx_PxJointLinearLimit const* self__pod) {
        physx::PxJointLinearLimit const* self_ = reinterpret_cast<physx::PxJointLinearLimit const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    void PxJointLinearLimit_delete(physx_PxJointLinearLimit* self__pod) {
        physx::PxJointLinearLimit* self_ = reinterpret_cast<physx::PxJointLinearLimit*>(self__pod);
        delete self_;
    }

    physx_PxJointLinearLimitPair PxJointLinearLimitPair_new(physx_PxTolerancesScale const* scale_pod, float lowerLimit, float upperLimit, float contactDist_deprecated) {
        physx::PxTolerancesScale const& scale = reinterpret_cast<physx::PxTolerancesScale const&>(*scale_pod);
        PxJointLinearLimitPair return_val(scale, lowerLimit, upperLimit, contactDist_deprecated);
        physx_PxJointLinearLimitPair return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxJointLinearLimitPair PxJointLinearLimitPair_new_1(float lowerLimit, float upperLimit, physx_PxSpring const* spring_pod) {
        physx::PxSpring const& spring = reinterpret_cast<physx::PxSpring const&>(*spring_pod);
        PxJointLinearLimitPair return_val(lowerLimit, upperLimit, spring);
        physx_PxJointLinearLimitPair return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxJointLinearLimitPair_isValid(physx_PxJointLinearLimitPair const* self__pod) {
        physx::PxJointLinearLimitPair const* self_ = reinterpret_cast<physx::PxJointLinearLimitPair const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    void PxJointLinearLimitPair_delete(physx_PxJointLinearLimitPair* self__pod) {
        physx::PxJointLinearLimitPair* self_ = reinterpret_cast<physx::PxJointLinearLimitPair*>(self__pod);
        delete self_;
    }

    physx_PxJointAngularLimitPair PxJointAngularLimitPair_new(float lowerLimit, float upperLimit, float contactDist_deprecated) {
        PxJointAngularLimitPair return_val(lowerLimit, upperLimit, contactDist_deprecated);
        physx_PxJointAngularLimitPair return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxJointAngularLimitPair PxJointAngularLimitPair_new_1(float lowerLimit, float upperLimit, physx_PxSpring const* spring_pod) {
        physx::PxSpring const& spring = reinterpret_cast<physx::PxSpring const&>(*spring_pod);
        PxJointAngularLimitPair return_val(lowerLimit, upperLimit, spring);
        physx_PxJointAngularLimitPair return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxJointAngularLimitPair_isValid(physx_PxJointAngularLimitPair const* self__pod) {
        physx::PxJointAngularLimitPair const* self_ = reinterpret_cast<physx::PxJointAngularLimitPair const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    void PxJointAngularLimitPair_delete(physx_PxJointAngularLimitPair* self__pod) {
        physx::PxJointAngularLimitPair* self_ = reinterpret_cast<physx::PxJointAngularLimitPair*>(self__pod);
        delete self_;
    }

    physx_PxJointLimitCone PxJointLimitCone_new(float yLimitAngle, float zLimitAngle, float contactDist_deprecated) {
        PxJointLimitCone return_val(yLimitAngle, zLimitAngle, contactDist_deprecated);
        physx_PxJointLimitCone return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxJointLimitCone PxJointLimitCone_new_1(float yLimitAngle, float zLimitAngle, physx_PxSpring const* spring_pod) {
        physx::PxSpring const& spring = reinterpret_cast<physx::PxSpring const&>(*spring_pod);
        PxJointLimitCone return_val(yLimitAngle, zLimitAngle, spring);
        physx_PxJointLimitCone return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxJointLimitCone_isValid(physx_PxJointLimitCone const* self__pod) {
        physx::PxJointLimitCone const* self_ = reinterpret_cast<physx::PxJointLimitCone const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    void PxJointLimitCone_delete(physx_PxJointLimitCone* self__pod) {
        physx::PxJointLimitCone* self_ = reinterpret_cast<physx::PxJointLimitCone*>(self__pod);
        delete self_;
    }

    physx_PxJointLimitPyramid PxJointLimitPyramid_new(float yLimitAngleMin, float yLimitAngleMax, float zLimitAngleMin, float zLimitAngleMax, float contactDist_deprecated) {
        PxJointLimitPyramid return_val(yLimitAngleMin, yLimitAngleMax, zLimitAngleMin, zLimitAngleMax, contactDist_deprecated);
        physx_PxJointLimitPyramid return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxJointLimitPyramid PxJointLimitPyramid_new_1(float yLimitAngleMin, float yLimitAngleMax, float zLimitAngleMin, float zLimitAngleMax, physx_PxSpring const* spring_pod) {
        physx::PxSpring const& spring = reinterpret_cast<physx::PxSpring const&>(*spring_pod);
        PxJointLimitPyramid return_val(yLimitAngleMin, yLimitAngleMax, zLimitAngleMin, zLimitAngleMax, spring);
        physx_PxJointLimitPyramid return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxJointLimitPyramid_isValid(physx_PxJointLimitPyramid const* self__pod) {
        physx::PxJointLimitPyramid const* self_ = reinterpret_cast<physx::PxJointLimitPyramid const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    void PxJointLimitPyramid_delete(physx_PxJointLimitPyramid* self__pod) {
        physx::PxJointLimitPyramid* self_ = reinterpret_cast<physx::PxJointLimitPyramid*>(self__pod);
        delete self_;
    }

    physx_PxPrismaticJoint* phys_PxPrismaticJointCreate(physx_PxPhysics* physics_pod, physx_PxRigidActor* actor0_pod, physx_PxTransform const* localFrame0_pod, physx_PxRigidActor* actor1_pod, physx_PxTransform const* localFrame1_pod) {
        physx::PxPhysics& physics = reinterpret_cast<physx::PxPhysics&>(*physics_pod);
        physx::PxRigidActor* actor0 = reinterpret_cast<physx::PxRigidActor*>(actor0_pod);
        physx::PxTransform const& localFrame0 = reinterpret_cast<physx::PxTransform const&>(*localFrame0_pod);
        physx::PxRigidActor* actor1 = reinterpret_cast<physx::PxRigidActor*>(actor1_pod);
        physx::PxTransform const& localFrame1 = reinterpret_cast<physx::PxTransform const&>(*localFrame1_pod);
        physx::PxPrismaticJoint* return_val = PxPrismaticJointCreate(physics, actor0, localFrame0, actor1, localFrame1);
        auto return_val_pod = reinterpret_cast<physx_PxPrismaticJoint*>(return_val);
        return return_val_pod;
    }

    float PxPrismaticJoint_getPosition(physx_PxPrismaticJoint const* self__pod) {
        physx::PxPrismaticJoint const* self_ = reinterpret_cast<physx::PxPrismaticJoint const*>(self__pod);
        float return_val = self_->getPosition();
        return return_val;
    }

    float PxPrismaticJoint_getVelocity(physx_PxPrismaticJoint const* self__pod) {
        physx::PxPrismaticJoint const* self_ = reinterpret_cast<physx::PxPrismaticJoint const*>(self__pod);
        float return_val = self_->getVelocity();
        return return_val;
    }

    void PxPrismaticJoint_setLimit(physx_PxPrismaticJoint* self__pod, physx_PxJointLinearLimitPair const* anon_param0_pod) {
        physx::PxPrismaticJoint* self_ = reinterpret_cast<physx::PxPrismaticJoint*>(self__pod);
        physx::PxJointLinearLimitPair const& anon_param0 = reinterpret_cast<physx::PxJointLinearLimitPair const&>(*anon_param0_pod);
        self_->setLimit(anon_param0);
    }

    physx_PxJointLinearLimitPair PxPrismaticJoint_getLimit(physx_PxPrismaticJoint const* self__pod) {
        physx::PxPrismaticJoint const* self_ = reinterpret_cast<physx::PxPrismaticJoint const*>(self__pod);
        physx::PxJointLinearLimitPair return_val = self_->getLimit();
        physx_PxJointLinearLimitPair return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxPrismaticJoint_setPrismaticJointFlags(physx_PxPrismaticJoint* self__pod, PxPrismaticJointFlags flags_pod) {
        physx::PxPrismaticJoint* self_ = reinterpret_cast<physx::PxPrismaticJoint*>(self__pod);
        auto flags = physx::PxPrismaticJointFlags(flags_pod);
        self_->setPrismaticJointFlags(flags);
    }

    void PxPrismaticJoint_setPrismaticJointFlag(physx_PxPrismaticJoint* self__pod, PxPrismaticJointFlag flag_pod, bool value) {
        physx::PxPrismaticJoint* self_ = reinterpret_cast<physx::PxPrismaticJoint*>(self__pod);
        auto flag = static_cast<physx::PxPrismaticJointFlag::Enum>(flag_pod);
        self_->setPrismaticJointFlag(flag, value);
    }

    PxPrismaticJointFlags PxPrismaticJoint_getPrismaticJointFlags(physx_PxPrismaticJoint const* self__pod) {
        physx::PxPrismaticJoint const* self_ = reinterpret_cast<physx::PxPrismaticJoint const*>(self__pod);
        physx::PxPrismaticJointFlags return_val = self_->getPrismaticJointFlags();
        PxPrismaticJointFlags return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    char const* PxPrismaticJoint_getConcreteTypeName(physx_PxPrismaticJoint const* self__pod) {
        physx::PxPrismaticJoint const* self_ = reinterpret_cast<physx::PxPrismaticJoint const*>(self__pod);
        char const* return_val = self_->getConcreteTypeName();
        return return_val;
    }

    physx_PxRevoluteJoint* phys_PxRevoluteJointCreate(physx_PxPhysics* physics_pod, physx_PxRigidActor* actor0_pod, physx_PxTransform const* localFrame0_pod, physx_PxRigidActor* actor1_pod, physx_PxTransform const* localFrame1_pod) {
        physx::PxPhysics& physics = reinterpret_cast<physx::PxPhysics&>(*physics_pod);
        physx::PxRigidActor* actor0 = reinterpret_cast<physx::PxRigidActor*>(actor0_pod);
        physx::PxTransform const& localFrame0 = reinterpret_cast<physx::PxTransform const&>(*localFrame0_pod);
        physx::PxRigidActor* actor1 = reinterpret_cast<physx::PxRigidActor*>(actor1_pod);
        physx::PxTransform const& localFrame1 = reinterpret_cast<physx::PxTransform const&>(*localFrame1_pod);
        physx::PxRevoluteJoint* return_val = PxRevoluteJointCreate(physics, actor0, localFrame0, actor1, localFrame1);
        auto return_val_pod = reinterpret_cast<physx_PxRevoluteJoint*>(return_val);
        return return_val_pod;
    }

    float PxRevoluteJoint_getAngle(physx_PxRevoluteJoint const* self__pod) {
        physx::PxRevoluteJoint const* self_ = reinterpret_cast<physx::PxRevoluteJoint const*>(self__pod);
        float return_val = self_->getAngle();
        return return_val;
    }

    float PxRevoluteJoint_getVelocity(physx_PxRevoluteJoint const* self__pod) {
        physx::PxRevoluteJoint const* self_ = reinterpret_cast<physx::PxRevoluteJoint const*>(self__pod);
        float return_val = self_->getVelocity();
        return return_val;
    }

    void PxRevoluteJoint_setLimit(physx_PxRevoluteJoint* self__pod, physx_PxJointAngularLimitPair const* limits_pod) {
        physx::PxRevoluteJoint* self_ = reinterpret_cast<physx::PxRevoluteJoint*>(self__pod);
        physx::PxJointAngularLimitPair const& limits = reinterpret_cast<physx::PxJointAngularLimitPair const&>(*limits_pod);
        self_->setLimit(limits);
    }

    physx_PxJointAngularLimitPair PxRevoluteJoint_getLimit(physx_PxRevoluteJoint const* self__pod) {
        physx::PxRevoluteJoint const* self_ = reinterpret_cast<physx::PxRevoluteJoint const*>(self__pod);
        physx::PxJointAngularLimitPair return_val = self_->getLimit();
        physx_PxJointAngularLimitPair return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxRevoluteJoint_setDriveVelocity(physx_PxRevoluteJoint* self__pod, float velocity, bool autowake) {
        physx::PxRevoluteJoint* self_ = reinterpret_cast<physx::PxRevoluteJoint*>(self__pod);
        self_->setDriveVelocity(velocity, autowake);
    }

    float PxRevoluteJoint_getDriveVelocity(physx_PxRevoluteJoint const* self__pod) {
        physx::PxRevoluteJoint const* self_ = reinterpret_cast<physx::PxRevoluteJoint const*>(self__pod);
        float return_val = self_->getDriveVelocity();
        return return_val;
    }

    void PxRevoluteJoint_setDriveForceLimit(physx_PxRevoluteJoint* self__pod, float limit) {
        physx::PxRevoluteJoint* self_ = reinterpret_cast<physx::PxRevoluteJoint*>(self__pod);
        self_->setDriveForceLimit(limit);
    }

    float PxRevoluteJoint_getDriveForceLimit(physx_PxRevoluteJoint const* self__pod) {
        physx::PxRevoluteJoint const* self_ = reinterpret_cast<physx::PxRevoluteJoint const*>(self__pod);
        float return_val = self_->getDriveForceLimit();
        return return_val;
    }

    void PxRevoluteJoint_setDriveGearRatio(physx_PxRevoluteJoint* self__pod, float ratio) {
        physx::PxRevoluteJoint* self_ = reinterpret_cast<physx::PxRevoluteJoint*>(self__pod);
        self_->setDriveGearRatio(ratio);
    }

    float PxRevoluteJoint_getDriveGearRatio(physx_PxRevoluteJoint const* self__pod) {
        physx::PxRevoluteJoint const* self_ = reinterpret_cast<physx::PxRevoluteJoint const*>(self__pod);
        float return_val = self_->getDriveGearRatio();
        return return_val;
    }

    void PxRevoluteJoint_setRevoluteJointFlags(physx_PxRevoluteJoint* self__pod, PxRevoluteJointFlags flags_pod) {
        physx::PxRevoluteJoint* self_ = reinterpret_cast<physx::PxRevoluteJoint*>(self__pod);
        auto flags = physx::PxRevoluteJointFlags(flags_pod);
        self_->setRevoluteJointFlags(flags);
    }

    void PxRevoluteJoint_setRevoluteJointFlag(physx_PxRevoluteJoint* self__pod, PxRevoluteJointFlag flag_pod, bool value) {
        physx::PxRevoluteJoint* self_ = reinterpret_cast<physx::PxRevoluteJoint*>(self__pod);
        auto flag = static_cast<physx::PxRevoluteJointFlag::Enum>(flag_pod);
        self_->setRevoluteJointFlag(flag, value);
    }

    PxRevoluteJointFlags PxRevoluteJoint_getRevoluteJointFlags(physx_PxRevoluteJoint const* self__pod) {
        physx::PxRevoluteJoint const* self_ = reinterpret_cast<physx::PxRevoluteJoint const*>(self__pod);
        physx::PxRevoluteJointFlags return_val = self_->getRevoluteJointFlags();
        PxRevoluteJointFlags return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    char const* PxRevoluteJoint_getConcreteTypeName(physx_PxRevoluteJoint const* self__pod) {
        physx::PxRevoluteJoint const* self_ = reinterpret_cast<physx::PxRevoluteJoint const*>(self__pod);
        char const* return_val = self_->getConcreteTypeName();
        return return_val;
    }

    physx_PxSphericalJoint* phys_PxSphericalJointCreate(physx_PxPhysics* physics_pod, physx_PxRigidActor* actor0_pod, physx_PxTransform const* localFrame0_pod, physx_PxRigidActor* actor1_pod, physx_PxTransform const* localFrame1_pod) {
        physx::PxPhysics& physics = reinterpret_cast<physx::PxPhysics&>(*physics_pod);
        physx::PxRigidActor* actor0 = reinterpret_cast<physx::PxRigidActor*>(actor0_pod);
        physx::PxTransform const& localFrame0 = reinterpret_cast<physx::PxTransform const&>(*localFrame0_pod);
        physx::PxRigidActor* actor1 = reinterpret_cast<physx::PxRigidActor*>(actor1_pod);
        physx::PxTransform const& localFrame1 = reinterpret_cast<physx::PxTransform const&>(*localFrame1_pod);
        physx::PxSphericalJoint* return_val = PxSphericalJointCreate(physics, actor0, localFrame0, actor1, localFrame1);
        auto return_val_pod = reinterpret_cast<physx_PxSphericalJoint*>(return_val);
        return return_val_pod;
    }

    physx_PxJointLimitCone PxSphericalJoint_getLimitCone(physx_PxSphericalJoint const* self__pod) {
        physx::PxSphericalJoint const* self_ = reinterpret_cast<physx::PxSphericalJoint const*>(self__pod);
        physx::PxJointLimitCone return_val = self_->getLimitCone();
        physx_PxJointLimitCone return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxSphericalJoint_setLimitCone(physx_PxSphericalJoint* self__pod, physx_PxJointLimitCone const* limit_pod) {
        physx::PxSphericalJoint* self_ = reinterpret_cast<physx::PxSphericalJoint*>(self__pod);
        physx::PxJointLimitCone const& limit = reinterpret_cast<physx::PxJointLimitCone const&>(*limit_pod);
        self_->setLimitCone(limit);
    }

    float PxSphericalJoint_getSwingYAngle(physx_PxSphericalJoint const* self__pod) {
        physx::PxSphericalJoint const* self_ = reinterpret_cast<physx::PxSphericalJoint const*>(self__pod);
        float return_val = self_->getSwingYAngle();
        return return_val;
    }

    float PxSphericalJoint_getSwingZAngle(physx_PxSphericalJoint const* self__pod) {
        physx::PxSphericalJoint const* self_ = reinterpret_cast<physx::PxSphericalJoint const*>(self__pod);
        float return_val = self_->getSwingZAngle();
        return return_val;
    }

    void PxSphericalJoint_setSphericalJointFlags(physx_PxSphericalJoint* self__pod, PxSphericalJointFlags flags_pod) {
        physx::PxSphericalJoint* self_ = reinterpret_cast<physx::PxSphericalJoint*>(self__pod);
        auto flags = physx::PxSphericalJointFlags(flags_pod);
        self_->setSphericalJointFlags(flags);
    }

    void PxSphericalJoint_setSphericalJointFlag(physx_PxSphericalJoint* self__pod, PxSphericalJointFlag flag_pod, bool value) {
        physx::PxSphericalJoint* self_ = reinterpret_cast<physx::PxSphericalJoint*>(self__pod);
        auto flag = static_cast<physx::PxSphericalJointFlag::Enum>(flag_pod);
        self_->setSphericalJointFlag(flag, value);
    }

    PxSphericalJointFlags PxSphericalJoint_getSphericalJointFlags(physx_PxSphericalJoint const* self__pod) {
        physx::PxSphericalJoint const* self_ = reinterpret_cast<physx::PxSphericalJoint const*>(self__pod);
        physx::PxSphericalJointFlags return_val = self_->getSphericalJointFlags();
        PxSphericalJointFlags return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    char const* PxSphericalJoint_getConcreteTypeName(physx_PxSphericalJoint const* self__pod) {
        physx::PxSphericalJoint const* self_ = reinterpret_cast<physx::PxSphericalJoint const*>(self__pod);
        char const* return_val = self_->getConcreteTypeName();
        return return_val;
    }

    physx_PxD6Joint* phys_PxD6JointCreate(physx_PxPhysics* physics_pod, physx_PxRigidActor* actor0_pod, physx_PxTransform const* localFrame0_pod, physx_PxRigidActor* actor1_pod, physx_PxTransform const* localFrame1_pod) {
        physx::PxPhysics& physics = reinterpret_cast<physx::PxPhysics&>(*physics_pod);
        physx::PxRigidActor* actor0 = reinterpret_cast<physx::PxRigidActor*>(actor0_pod);
        physx::PxTransform const& localFrame0 = reinterpret_cast<physx::PxTransform const&>(*localFrame0_pod);
        physx::PxRigidActor* actor1 = reinterpret_cast<physx::PxRigidActor*>(actor1_pod);
        physx::PxTransform const& localFrame1 = reinterpret_cast<physx::PxTransform const&>(*localFrame1_pod);
        physx::PxD6Joint* return_val = PxD6JointCreate(physics, actor0, localFrame0, actor1, localFrame1);
        auto return_val_pod = reinterpret_cast<physx_PxD6Joint*>(return_val);
        return return_val_pod;
    }

    physx_PxD6JointDrive PxD6JointDrive_new() {
        PxD6JointDrive return_val;
        physx_PxD6JointDrive return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxD6JointDrive PxD6JointDrive_new_1(float driveStiffness, float driveDamping, float driveForceLimit, bool isAcceleration) {
        PxD6JointDrive return_val(driveStiffness, driveDamping, driveForceLimit, isAcceleration);
        physx_PxD6JointDrive return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxD6JointDrive_isValid(physx_PxD6JointDrive const* self__pod) {
        physx::PxD6JointDrive const* self_ = reinterpret_cast<physx::PxD6JointDrive const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    void PxD6Joint_setMotion(physx_PxD6Joint* self__pod, PxD6Axis axis_pod, PxD6Motion type_pod) {
        physx::PxD6Joint* self_ = reinterpret_cast<physx::PxD6Joint*>(self__pod);
        auto axis = static_cast<physx::PxD6Axis::Enum>(axis_pod);
        auto type = static_cast<physx::PxD6Motion::Enum>(type_pod);
        self_->setMotion(axis, type);
    }

    PxD6Motion PxD6Joint_getMotion(physx_PxD6Joint const* self__pod, PxD6Axis axis_pod) {
        physx::PxD6Joint const* self_ = reinterpret_cast<physx::PxD6Joint const*>(self__pod);
        auto axis = static_cast<physx::PxD6Axis::Enum>(axis_pod);
        physx::PxD6Motion::Enum return_val = self_->getMotion(axis);
        PxD6Motion return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    float PxD6Joint_getTwistAngle(physx_PxD6Joint const* self__pod) {
        physx::PxD6Joint const* self_ = reinterpret_cast<physx::PxD6Joint const*>(self__pod);
        float return_val = self_->getTwistAngle();
        return return_val;
    }

    float PxD6Joint_getSwingYAngle(physx_PxD6Joint const* self__pod) {
        physx::PxD6Joint const* self_ = reinterpret_cast<physx::PxD6Joint const*>(self__pod);
        float return_val = self_->getSwingYAngle();
        return return_val;
    }

    float PxD6Joint_getSwingZAngle(physx_PxD6Joint const* self__pod) {
        physx::PxD6Joint const* self_ = reinterpret_cast<physx::PxD6Joint const*>(self__pod);
        float return_val = self_->getSwingZAngle();
        return return_val;
    }

    void PxD6Joint_setDistanceLimit(physx_PxD6Joint* self__pod, physx_PxJointLinearLimit const* limit_pod) {
        physx::PxD6Joint* self_ = reinterpret_cast<physx::PxD6Joint*>(self__pod);
        physx::PxJointLinearLimit const& limit = reinterpret_cast<physx::PxJointLinearLimit const&>(*limit_pod);
        self_->setDistanceLimit(limit);
    }

    physx_PxJointLinearLimit PxD6Joint_getDistanceLimit(physx_PxD6Joint const* self__pod) {
        physx::PxD6Joint const* self_ = reinterpret_cast<physx::PxD6Joint const*>(self__pod);
        physx::PxJointLinearLimit return_val = self_->getDistanceLimit();
        physx_PxJointLinearLimit return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxD6Joint_setLinearLimit(physx_PxD6Joint* self__pod, PxD6Axis axis_pod, physx_PxJointLinearLimitPair const* limit_pod) {
        physx::PxD6Joint* self_ = reinterpret_cast<physx::PxD6Joint*>(self__pod);
        auto axis = static_cast<physx::PxD6Axis::Enum>(axis_pod);
        physx::PxJointLinearLimitPair const& limit = reinterpret_cast<physx::PxJointLinearLimitPair const&>(*limit_pod);
        self_->setLinearLimit(axis, limit);
    }

    physx_PxJointLinearLimitPair PxD6Joint_getLinearLimit(physx_PxD6Joint const* self__pod, PxD6Axis axis_pod) {
        physx::PxD6Joint const* self_ = reinterpret_cast<physx::PxD6Joint const*>(self__pod);
        auto axis = static_cast<physx::PxD6Axis::Enum>(axis_pod);
        physx::PxJointLinearLimitPair return_val = self_->getLinearLimit(axis);
        physx_PxJointLinearLimitPair return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxD6Joint_setTwistLimit(physx_PxD6Joint* self__pod, physx_PxJointAngularLimitPair const* limit_pod) {
        physx::PxD6Joint* self_ = reinterpret_cast<physx::PxD6Joint*>(self__pod);
        physx::PxJointAngularLimitPair const& limit = reinterpret_cast<physx::PxJointAngularLimitPair const&>(*limit_pod);
        self_->setTwistLimit(limit);
    }

    physx_PxJointAngularLimitPair PxD6Joint_getTwistLimit(physx_PxD6Joint const* self__pod) {
        physx::PxD6Joint const* self_ = reinterpret_cast<physx::PxD6Joint const*>(self__pod);
        physx::PxJointAngularLimitPair return_val = self_->getTwistLimit();
        physx_PxJointAngularLimitPair return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxD6Joint_setSwingLimit(physx_PxD6Joint* self__pod, physx_PxJointLimitCone const* limit_pod) {
        physx::PxD6Joint* self_ = reinterpret_cast<physx::PxD6Joint*>(self__pod);
        physx::PxJointLimitCone const& limit = reinterpret_cast<physx::PxJointLimitCone const&>(*limit_pod);
        self_->setSwingLimit(limit);
    }

    physx_PxJointLimitCone PxD6Joint_getSwingLimit(physx_PxD6Joint const* self__pod) {
        physx::PxD6Joint const* self_ = reinterpret_cast<physx::PxD6Joint const*>(self__pod);
        physx::PxJointLimitCone return_val = self_->getSwingLimit();
        physx_PxJointLimitCone return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxD6Joint_setPyramidSwingLimit(physx_PxD6Joint* self__pod, physx_PxJointLimitPyramid const* limit_pod) {
        physx::PxD6Joint* self_ = reinterpret_cast<physx::PxD6Joint*>(self__pod);
        physx::PxJointLimitPyramid const& limit = reinterpret_cast<physx::PxJointLimitPyramid const&>(*limit_pod);
        self_->setPyramidSwingLimit(limit);
    }

    physx_PxJointLimitPyramid PxD6Joint_getPyramidSwingLimit(physx_PxD6Joint const* self__pod) {
        physx::PxD6Joint const* self_ = reinterpret_cast<physx::PxD6Joint const*>(self__pod);
        physx::PxJointLimitPyramid return_val = self_->getPyramidSwingLimit();
        physx_PxJointLimitPyramid return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxD6Joint_setDrive(physx_PxD6Joint* self__pod, PxD6Drive index_pod, physx_PxD6JointDrive const* drive_pod) {
        physx::PxD6Joint* self_ = reinterpret_cast<physx::PxD6Joint*>(self__pod);
        auto index = static_cast<physx::PxD6Drive::Enum>(index_pod);
        physx::PxD6JointDrive const& drive = reinterpret_cast<physx::PxD6JointDrive const&>(*drive_pod);
        self_->setDrive(index, drive);
    }

    physx_PxD6JointDrive PxD6Joint_getDrive(physx_PxD6Joint const* self__pod, PxD6Drive index_pod) {
        physx::PxD6Joint const* self_ = reinterpret_cast<physx::PxD6Joint const*>(self__pod);
        auto index = static_cast<physx::PxD6Drive::Enum>(index_pod);
        physx::PxD6JointDrive return_val = self_->getDrive(index);
        physx_PxD6JointDrive return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxD6Joint_setDrivePosition(physx_PxD6Joint* self__pod, physx_PxTransform const* pose_pod, bool autowake) {
        physx::PxD6Joint* self_ = reinterpret_cast<physx::PxD6Joint*>(self__pod);
        physx::PxTransform const& pose = reinterpret_cast<physx::PxTransform const&>(*pose_pod);
        self_->setDrivePosition(pose, autowake);
    }

    physx_PxTransform PxD6Joint_getDrivePosition(physx_PxD6Joint const* self__pod) {
        physx::PxD6Joint const* self_ = reinterpret_cast<physx::PxD6Joint const*>(self__pod);
        physx::PxTransform return_val = self_->getDrivePosition();
        physx_PxTransform return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxD6Joint_setDriveVelocity(physx_PxD6Joint* self__pod, physx_PxVec3 const* linear_pod, physx_PxVec3 const* angular_pod, bool autowake) {
        physx::PxD6Joint* self_ = reinterpret_cast<physx::PxD6Joint*>(self__pod);
        physx::PxVec3 const& linear = reinterpret_cast<physx::PxVec3 const&>(*linear_pod);
        physx::PxVec3 const& angular = reinterpret_cast<physx::PxVec3 const&>(*angular_pod);
        self_->setDriveVelocity(linear, angular, autowake);
    }

    void PxD6Joint_getDriveVelocity(physx_PxD6Joint const* self__pod, physx_PxVec3* linear_pod, physx_PxVec3* angular_pod) {
        physx::PxD6Joint const* self_ = reinterpret_cast<physx::PxD6Joint const*>(self__pod);
        physx::PxVec3& linear = reinterpret_cast<physx::PxVec3&>(*linear_pod);
        physx::PxVec3& angular = reinterpret_cast<physx::PxVec3&>(*angular_pod);
        self_->getDriveVelocity(linear, angular);
    }

    void PxD6Joint_setProjectionLinearTolerance(physx_PxD6Joint* self__pod, float tolerance) {
        physx::PxD6Joint* self_ = reinterpret_cast<physx::PxD6Joint*>(self__pod);
        self_->setProjectionLinearTolerance(tolerance);
    }

    float PxD6Joint_getProjectionLinearTolerance(physx_PxD6Joint const* self__pod) {
        physx::PxD6Joint const* self_ = reinterpret_cast<physx::PxD6Joint const*>(self__pod);
        float return_val = self_->getProjectionLinearTolerance();
        return return_val;
    }

    void PxD6Joint_setProjectionAngularTolerance(physx_PxD6Joint* self__pod, float tolerance) {
        physx::PxD6Joint* self_ = reinterpret_cast<physx::PxD6Joint*>(self__pod);
        self_->setProjectionAngularTolerance(tolerance);
    }

    float PxD6Joint_getProjectionAngularTolerance(physx_PxD6Joint const* self__pod) {
        physx::PxD6Joint const* self_ = reinterpret_cast<physx::PxD6Joint const*>(self__pod);
        float return_val = self_->getProjectionAngularTolerance();
        return return_val;
    }

    char const* PxD6Joint_getConcreteTypeName(physx_PxD6Joint const* self__pod) {
        physx::PxD6Joint const* self_ = reinterpret_cast<physx::PxD6Joint const*>(self__pod);
        char const* return_val = self_->getConcreteTypeName();
        return return_val;
    }

    physx_PxGearJoint* phys_PxGearJointCreate(physx_PxPhysics* physics_pod, physx_PxRigidActor* actor0_pod, physx_PxTransform const* localFrame0_pod, physx_PxRigidActor* actor1_pod, physx_PxTransform const* localFrame1_pod) {
        physx::PxPhysics& physics = reinterpret_cast<physx::PxPhysics&>(*physics_pod);
        physx::PxRigidActor* actor0 = reinterpret_cast<physx::PxRigidActor*>(actor0_pod);
        physx::PxTransform const& localFrame0 = reinterpret_cast<physx::PxTransform const&>(*localFrame0_pod);
        physx::PxRigidActor* actor1 = reinterpret_cast<physx::PxRigidActor*>(actor1_pod);
        physx::PxTransform const& localFrame1 = reinterpret_cast<physx::PxTransform const&>(*localFrame1_pod);
        physx::PxGearJoint* return_val = PxGearJointCreate(physics, actor0, localFrame0, actor1, localFrame1);
        auto return_val_pod = reinterpret_cast<physx_PxGearJoint*>(return_val);
        return return_val_pod;
    }

    bool PxGearJoint_setHinges(physx_PxGearJoint* self__pod, physx_PxBase const* hinge0_pod, physx_PxBase const* hinge1_pod) {
        physx::PxGearJoint* self_ = reinterpret_cast<physx::PxGearJoint*>(self__pod);
        physx::PxBase const* hinge0 = reinterpret_cast<physx::PxBase const*>(hinge0_pod);
        physx::PxBase const* hinge1 = reinterpret_cast<physx::PxBase const*>(hinge1_pod);
        bool return_val = self_->setHinges(hinge0, hinge1);
        return return_val;
    }

    void PxGearJoint_setGearRatio(physx_PxGearJoint* self__pod, float ratio) {
        physx::PxGearJoint* self_ = reinterpret_cast<physx::PxGearJoint*>(self__pod);
        self_->setGearRatio(ratio);
    }

    float PxGearJoint_getGearRatio(physx_PxGearJoint const* self__pod) {
        physx::PxGearJoint const* self_ = reinterpret_cast<physx::PxGearJoint const*>(self__pod);
        float return_val = self_->getGearRatio();
        return return_val;
    }

    char const* PxGearJoint_getConcreteTypeName(physx_PxGearJoint const* self__pod) {
        physx::PxGearJoint const* self_ = reinterpret_cast<physx::PxGearJoint const*>(self__pod);
        char const* return_val = self_->getConcreteTypeName();
        return return_val;
    }

    physx_PxRackAndPinionJoint* phys_PxRackAndPinionJointCreate(physx_PxPhysics* physics_pod, physx_PxRigidActor* actor0_pod, physx_PxTransform const* localFrame0_pod, physx_PxRigidActor* actor1_pod, physx_PxTransform const* localFrame1_pod) {
        physx::PxPhysics& physics = reinterpret_cast<physx::PxPhysics&>(*physics_pod);
        physx::PxRigidActor* actor0 = reinterpret_cast<physx::PxRigidActor*>(actor0_pod);
        physx::PxTransform const& localFrame0 = reinterpret_cast<physx::PxTransform const&>(*localFrame0_pod);
        physx::PxRigidActor* actor1 = reinterpret_cast<physx::PxRigidActor*>(actor1_pod);
        physx::PxTransform const& localFrame1 = reinterpret_cast<physx::PxTransform const&>(*localFrame1_pod);
        physx::PxRackAndPinionJoint* return_val = PxRackAndPinionJointCreate(physics, actor0, localFrame0, actor1, localFrame1);
        auto return_val_pod = reinterpret_cast<physx_PxRackAndPinionJoint*>(return_val);
        return return_val_pod;
    }

    bool PxRackAndPinionJoint_setJoints(physx_PxRackAndPinionJoint* self__pod, physx_PxBase const* hinge_pod, physx_PxBase const* prismatic_pod) {
        physx::PxRackAndPinionJoint* self_ = reinterpret_cast<physx::PxRackAndPinionJoint*>(self__pod);
        physx::PxBase const* hinge = reinterpret_cast<physx::PxBase const*>(hinge_pod);
        physx::PxBase const* prismatic = reinterpret_cast<physx::PxBase const*>(prismatic_pod);
        bool return_val = self_->setJoints(hinge, prismatic);
        return return_val;
    }

    void PxRackAndPinionJoint_setRatio(physx_PxRackAndPinionJoint* self__pod, float ratio) {
        physx::PxRackAndPinionJoint* self_ = reinterpret_cast<physx::PxRackAndPinionJoint*>(self__pod);
        self_->setRatio(ratio);
    }

    float PxRackAndPinionJoint_getRatio(physx_PxRackAndPinionJoint const* self__pod) {
        physx::PxRackAndPinionJoint const* self_ = reinterpret_cast<physx::PxRackAndPinionJoint const*>(self__pod);
        float return_val = self_->getRatio();
        return return_val;
    }

    bool PxRackAndPinionJoint_setData(physx_PxRackAndPinionJoint* self__pod, uint32_t nbRackTeeth, uint32_t nbPinionTeeth, float rackLength) {
        physx::PxRackAndPinionJoint* self_ = reinterpret_cast<physx::PxRackAndPinionJoint*>(self__pod);
        bool return_val = self_->setData(nbRackTeeth, nbPinionTeeth, rackLength);
        return return_val;
    }

    char const* PxRackAndPinionJoint_getConcreteTypeName(physx_PxRackAndPinionJoint const* self__pod) {
        physx::PxRackAndPinionJoint const* self_ = reinterpret_cast<physx::PxRackAndPinionJoint const*>(self__pod);
        char const* return_val = self_->getConcreteTypeName();
        return return_val;
    }

    physx_PxGroupsMask* PxGroupsMask_new_alloc() {
        auto return_val = new physx::PxGroupsMask();
        auto return_val_pod = reinterpret_cast<physx_PxGroupsMask*>(return_val);
        return return_val_pod;
    }

    void PxGroupsMask_delete(physx_PxGroupsMask* self__pod) {
        physx::PxGroupsMask* self_ = reinterpret_cast<physx::PxGroupsMask*>(self__pod);
        delete self_;
    }

    PxFilterFlags phys_PxDefaultSimulationFilterShader(uint32_t attributes0, physx_PxFilterData filterData0_pod, uint32_t attributes1, physx_PxFilterData filterData1_pod, PxPairFlags* pairFlags_pod, void const* constantBlock, uint32_t constantBlockSize) {
        physx::PxFilterData filterData0;
        memcpy(&filterData0, &filterData0_pod, sizeof(filterData0));
        physx::PxFilterData filterData1;
        memcpy(&filterData1, &filterData1_pod, sizeof(filterData1));
        physx::PxPairFlags& pairFlags = reinterpret_cast<physx::PxPairFlags&>(*pairFlags_pod);
        physx::PxFilterFlags return_val = PxDefaultSimulationFilterShader(attributes0, filterData0, attributes1, filterData1, pairFlags, constantBlock, constantBlockSize);
        PxFilterFlags return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool phys_PxGetGroupCollisionFlag(uint16_t group1, uint16_t group2) {
        bool return_val = PxGetGroupCollisionFlag(group1, group2);
        return return_val;
    }

    void phys_PxSetGroupCollisionFlag(uint16_t group1, uint16_t group2, bool enable) {
        PxSetGroupCollisionFlag(group1, group2, enable);
    }

    uint16_t phys_PxGetGroup(physx_PxActor const* actor_pod) {
        physx::PxActor const& actor = reinterpret_cast<physx::PxActor const&>(*actor_pod);
        uint16_t return_val = PxGetGroup(actor);
        return return_val;
    }

    void phys_PxSetGroup(physx_PxActor* actor_pod, uint16_t collisionGroup) {
        physx::PxActor& actor = reinterpret_cast<physx::PxActor&>(*actor_pod);
        PxSetGroup(actor, collisionGroup);
    }

    void phys_PxGetFilterOps(PxFilterOp* op0_pod, PxFilterOp* op1_pod, PxFilterOp* op2_pod) {
        physx::PxFilterOp::Enum& op0 = reinterpret_cast<physx::PxFilterOp::Enum&>(*op0_pod);
        physx::PxFilterOp::Enum& op1 = reinterpret_cast<physx::PxFilterOp::Enum&>(*op1_pod);
        physx::PxFilterOp::Enum& op2 = reinterpret_cast<physx::PxFilterOp::Enum&>(*op2_pod);
        PxGetFilterOps(op0, op1, op2);
    }

    void phys_PxSetFilterOps(PxFilterOp const* op0_pod, PxFilterOp const* op1_pod, PxFilterOp const* op2_pod) {
        physx::PxFilterOp::Enum const& op0 = reinterpret_cast<physx::PxFilterOp::Enum const&>(*op0_pod);
        physx::PxFilterOp::Enum const& op1 = reinterpret_cast<physx::PxFilterOp::Enum const&>(*op1_pod);
        physx::PxFilterOp::Enum const& op2 = reinterpret_cast<physx::PxFilterOp::Enum const&>(*op2_pod);
        PxSetFilterOps(op0, op1, op2);
    }

    bool phys_PxGetFilterBool() {
        bool return_val = PxGetFilterBool();
        return return_val;
    }

    void phys_PxSetFilterBool(bool enable) {
        PxSetFilterBool(enable);
    }

    void phys_PxGetFilterConstants(physx_PxGroupsMask* c0_pod, physx_PxGroupsMask* c1_pod) {
        physx::PxGroupsMask& c0 = reinterpret_cast<physx::PxGroupsMask&>(*c0_pod);
        physx::PxGroupsMask& c1 = reinterpret_cast<physx::PxGroupsMask&>(*c1_pod);
        PxGetFilterConstants(c0, c1);
    }

    void phys_PxSetFilterConstants(physx_PxGroupsMask const* c0_pod, physx_PxGroupsMask const* c1_pod) {
        physx::PxGroupsMask const& c0 = reinterpret_cast<physx::PxGroupsMask const&>(*c0_pod);
        physx::PxGroupsMask const& c1 = reinterpret_cast<physx::PxGroupsMask const&>(*c1_pod);
        PxSetFilterConstants(c0, c1);
    }

    physx_PxGroupsMask phys_PxGetGroupsMask(physx_PxActor const* actor_pod) {
        physx::PxActor const& actor = reinterpret_cast<physx::PxActor const&>(*actor_pod);
        physx::PxGroupsMask return_val = PxGetGroupsMask(actor);
        physx_PxGroupsMask return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void phys_PxSetGroupsMask(physx_PxActor* actor_pod, physx_PxGroupsMask const* mask_pod) {
        physx::PxActor& actor = reinterpret_cast<physx::PxActor&>(*actor_pod);
        physx::PxGroupsMask const& mask = reinterpret_cast<physx::PxGroupsMask const&>(*mask_pod);
        PxSetGroupsMask(actor, mask);
    }

    physx_PxDefaultErrorCallback* PxDefaultErrorCallback_new_alloc() {
        auto return_val = new physx::PxDefaultErrorCallback();
        auto return_val_pod = reinterpret_cast<physx_PxDefaultErrorCallback*>(return_val);
        return return_val_pod;
    }

    void PxDefaultErrorCallback_delete(physx_PxDefaultErrorCallback* self__pod) {
        physx::PxDefaultErrorCallback* self_ = reinterpret_cast<physx::PxDefaultErrorCallback*>(self__pod);
        delete self_;
    }

    void PxDefaultErrorCallback_reportError(physx_PxDefaultErrorCallback* self__pod, PxErrorCode code_pod, char const* message, char const* file, int32_t line) {
        physx::PxDefaultErrorCallback* self_ = reinterpret_cast<physx::PxDefaultErrorCallback*>(self__pod);
        auto code = static_cast<physx::PxErrorCode::Enum>(code_pod);
        self_->reportError(code, message, file, line);
    }

    physx_PxShape* PxRigidActorExt_createExclusiveShape(physx_PxRigidActor* actor_pod, physx_PxGeometry const* geometry_pod, physx_PxMaterial* const* materials_pod, uint16_t materialCount, PxShapeFlags shapeFlags_pod) {
        physx::PxRigidActor& actor = reinterpret_cast<physx::PxRigidActor&>(*actor_pod);
        physx::PxGeometry const& geometry = reinterpret_cast<physx::PxGeometry const&>(*geometry_pod);
        physx::PxMaterial* const* materials = reinterpret_cast<physx::PxMaterial* const*>(materials_pod);
        auto shapeFlags = physx::PxShapeFlags(shapeFlags_pod);
        physx::PxShape* return_val = PxRigidActorExt::createExclusiveShape(actor, geometry, materials, materialCount, shapeFlags);
        auto return_val_pod = reinterpret_cast<physx_PxShape*>(return_val);
        return return_val_pod;
    }

    physx_PxShape* PxRigidActorExt_createExclusiveShape_1(physx_PxRigidActor* actor_pod, physx_PxGeometry const* geometry_pod, physx_PxMaterial const* material_pod, PxShapeFlags shapeFlags_pod) {
        physx::PxRigidActor& actor = reinterpret_cast<physx::PxRigidActor&>(*actor_pod);
        physx::PxGeometry const& geometry = reinterpret_cast<physx::PxGeometry const&>(*geometry_pod);
        physx::PxMaterial const& material = reinterpret_cast<physx::PxMaterial const&>(*material_pod);
        auto shapeFlags = physx::PxShapeFlags(shapeFlags_pod);
        physx::PxShape* return_val = PxRigidActorExt::createExclusiveShape(actor, geometry, material, shapeFlags);
        auto return_val_pod = reinterpret_cast<physx_PxShape*>(return_val);
        return return_val_pod;
    }

    physx_PxBounds3* PxRigidActorExt_getRigidActorShapeLocalBoundsList(physx_PxRigidActor const* actor_pod, uint32_t* numBounds_pod) {
        physx::PxRigidActor const& actor = reinterpret_cast<physx::PxRigidActor const&>(*actor_pod);
        uint32_t& numBounds = *numBounds_pod;
        physx::PxBounds3* return_val = PxRigidActorExt::getRigidActorShapeLocalBoundsList(actor, numBounds);
        auto return_val_pod = reinterpret_cast<physx_PxBounds3*>(return_val);
        return return_val_pod;
    }

    physx_PxBVH* PxRigidActorExt_createBVHFromActor(physx_PxPhysics* physics_pod, physx_PxRigidActor const* actor_pod) {
        physx::PxPhysics& physics = reinterpret_cast<physx::PxPhysics&>(*physics_pod);
        physx::PxRigidActor const& actor = reinterpret_cast<physx::PxRigidActor const&>(*actor_pod);
        physx::PxBVH* return_val = PxRigidActorExt::createBVHFromActor(physics, actor);
        auto return_val_pod = reinterpret_cast<physx_PxBVH*>(return_val);
        return return_val_pod;
    }

    physx_PxMassProperties PxMassProperties_new() {
        PxMassProperties return_val;
        physx_PxMassProperties return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxMassProperties PxMassProperties_new_1(float m, physx_PxMat33 const* inertiaT_pod, physx_PxVec3 const* com_pod) {
        physx::PxMat33 const& inertiaT = reinterpret_cast<physx::PxMat33 const&>(*inertiaT_pod);
        physx::PxVec3 const& com = reinterpret_cast<physx::PxVec3 const&>(*com_pod);
        PxMassProperties return_val(m, inertiaT, com);
        physx_PxMassProperties return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxMassProperties PxMassProperties_new_2(physx_PxGeometry const* geometry_pod) {
        physx::PxGeometry const& geometry = reinterpret_cast<physx::PxGeometry const&>(*geometry_pod);
        PxMassProperties return_val(geometry);
        physx_PxMassProperties return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxMassProperties_translate(physx_PxMassProperties* self__pod, physx_PxVec3 const* t_pod) {
        physx::PxMassProperties* self_ = reinterpret_cast<physx::PxMassProperties*>(self__pod);
        physx::PxVec3 const& t = reinterpret_cast<physx::PxVec3 const&>(*t_pod);
        self_->translate(t);
    }

    physx_PxVec3 PxMassProperties_getMassSpaceInertia(physx_PxMat33 const* inertia_pod, physx_PxQuat* massFrame_pod) {
        physx::PxMat33 const& inertia = reinterpret_cast<physx::PxMat33 const&>(*inertia_pod);
        physx::PxQuat& massFrame = reinterpret_cast<physx::PxQuat&>(*massFrame_pod);
        physx::PxVec3 return_val = PxMassProperties::getMassSpaceInertia(inertia, massFrame);
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxMat33 PxMassProperties_translateInertia(physx_PxMat33 const* inertia_pod, float mass, physx_PxVec3 const* t_pod) {
        physx::PxMat33 const& inertia = reinterpret_cast<physx::PxMat33 const&>(*inertia_pod);
        physx::PxVec3 const& t = reinterpret_cast<physx::PxVec3 const&>(*t_pod);
        physx::PxMat33 return_val = PxMassProperties::translateInertia(inertia, mass, t);
        physx_PxMat33 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxMat33 PxMassProperties_rotateInertia(physx_PxMat33 const* inertia_pod, physx_PxQuat const* q_pod) {
        physx::PxMat33 const& inertia = reinterpret_cast<physx::PxMat33 const&>(*inertia_pod);
        physx::PxQuat const& q = reinterpret_cast<physx::PxQuat const&>(*q_pod);
        physx::PxMat33 return_val = PxMassProperties::rotateInertia(inertia, q);
        physx_PxMat33 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxMat33 PxMassProperties_scaleInertia(physx_PxMat33 const* inertia_pod, physx_PxQuat const* scaleRotation_pod, physx_PxVec3 const* scale_pod) {
        physx::PxMat33 const& inertia = reinterpret_cast<physx::PxMat33 const&>(*inertia_pod);
        physx::PxQuat const& scaleRotation = reinterpret_cast<physx::PxQuat const&>(*scaleRotation_pod);
        physx::PxVec3 const& scale = reinterpret_cast<physx::PxVec3 const&>(*scale_pod);
        physx::PxMat33 return_val = PxMassProperties::scaleInertia(inertia, scaleRotation, scale);
        physx_PxMat33 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxMassProperties PxMassProperties_sum(physx_PxMassProperties const* props_pod, physx_PxTransform const* transforms_pod, uint32_t count) {
        physx::PxMassProperties const* props = reinterpret_cast<physx::PxMassProperties const*>(props_pod);
        physx::PxTransform const* transforms = reinterpret_cast<physx::PxTransform const*>(transforms_pod);
        physx::PxMassProperties return_val = PxMassProperties::sum(props, transforms, count);
        physx_PxMassProperties return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxRigidBodyExt_updateMassAndInertia(physx_PxRigidBody* body_pod, float const* shapeDensities, uint32_t shapeDensityCount, physx_PxVec3 const* massLocalPose_pod, bool includeNonSimShapes) {
        physx::PxRigidBody& body = reinterpret_cast<physx::PxRigidBody&>(*body_pod);
        physx::PxVec3 const* massLocalPose = reinterpret_cast<physx::PxVec3 const*>(massLocalPose_pod);
        bool return_val = PxRigidBodyExt::updateMassAndInertia(body, shapeDensities, shapeDensityCount, massLocalPose, includeNonSimShapes);
        return return_val;
    }

    bool PxRigidBodyExt_updateMassAndInertia_1(physx_PxRigidBody* body_pod, float density, physx_PxVec3 const* massLocalPose_pod, bool includeNonSimShapes) {
        physx::PxRigidBody& body = reinterpret_cast<physx::PxRigidBody&>(*body_pod);
        physx::PxVec3 const* massLocalPose = reinterpret_cast<physx::PxVec3 const*>(massLocalPose_pod);
        bool return_val = PxRigidBodyExt::updateMassAndInertia(body, density, massLocalPose, includeNonSimShapes);
        return return_val;
    }

    bool PxRigidBodyExt_setMassAndUpdateInertia(physx_PxRigidBody* body_pod, float const* shapeMasses, uint32_t shapeMassCount, physx_PxVec3 const* massLocalPose_pod, bool includeNonSimShapes) {
        physx::PxRigidBody& body = reinterpret_cast<physx::PxRigidBody&>(*body_pod);
        physx::PxVec3 const* massLocalPose = reinterpret_cast<physx::PxVec3 const*>(massLocalPose_pod);
        bool return_val = PxRigidBodyExt::setMassAndUpdateInertia(body, shapeMasses, shapeMassCount, massLocalPose, includeNonSimShapes);
        return return_val;
    }

    bool PxRigidBodyExt_setMassAndUpdateInertia_1(physx_PxRigidBody* body_pod, float mass, physx_PxVec3 const* massLocalPose_pod, bool includeNonSimShapes) {
        physx::PxRigidBody& body = reinterpret_cast<physx::PxRigidBody&>(*body_pod);
        physx::PxVec3 const* massLocalPose = reinterpret_cast<physx::PxVec3 const*>(massLocalPose_pod);
        bool return_val = PxRigidBodyExt::setMassAndUpdateInertia(body, mass, massLocalPose, includeNonSimShapes);
        return return_val;
    }

    physx_PxMassProperties PxRigidBodyExt_computeMassPropertiesFromShapes(physx_PxShape const* const* shapes_pod, uint32_t shapeCount) {
        physx::PxShape const* const* shapes = reinterpret_cast<physx::PxShape const* const*>(shapes_pod);
        physx::PxMassProperties return_val = PxRigidBodyExt::computeMassPropertiesFromShapes(shapes, shapeCount);
        physx_PxMassProperties return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxRigidBodyExt_addForceAtPos(physx_PxRigidBody* body_pod, physx_PxVec3 const* force_pod, physx_PxVec3 const* pos_pod, PxForceMode mode_pod, bool wakeup) {
        physx::PxRigidBody& body = reinterpret_cast<physx::PxRigidBody&>(*body_pod);
        physx::PxVec3 const& force = reinterpret_cast<physx::PxVec3 const&>(*force_pod);
        physx::PxVec3 const& pos = reinterpret_cast<physx::PxVec3 const&>(*pos_pod);
        auto mode = static_cast<physx::PxForceMode::Enum>(mode_pod);
        PxRigidBodyExt::addForceAtPos(body, force, pos, mode, wakeup);
    }

    void PxRigidBodyExt_addForceAtLocalPos(physx_PxRigidBody* body_pod, physx_PxVec3 const* force_pod, physx_PxVec3 const* pos_pod, PxForceMode mode_pod, bool wakeup) {
        physx::PxRigidBody& body = reinterpret_cast<physx::PxRigidBody&>(*body_pod);
        physx::PxVec3 const& force = reinterpret_cast<physx::PxVec3 const&>(*force_pod);
        physx::PxVec3 const& pos = reinterpret_cast<physx::PxVec3 const&>(*pos_pod);
        auto mode = static_cast<physx::PxForceMode::Enum>(mode_pod);
        PxRigidBodyExt::addForceAtLocalPos(body, force, pos, mode, wakeup);
    }

    void PxRigidBodyExt_addLocalForceAtPos(physx_PxRigidBody* body_pod, physx_PxVec3 const* force_pod, physx_PxVec3 const* pos_pod, PxForceMode mode_pod, bool wakeup) {
        physx::PxRigidBody& body = reinterpret_cast<physx::PxRigidBody&>(*body_pod);
        physx::PxVec3 const& force = reinterpret_cast<physx::PxVec3 const&>(*force_pod);
        physx::PxVec3 const& pos = reinterpret_cast<physx::PxVec3 const&>(*pos_pod);
        auto mode = static_cast<physx::PxForceMode::Enum>(mode_pod);
        PxRigidBodyExt::addLocalForceAtPos(body, force, pos, mode, wakeup);
    }

    void PxRigidBodyExt_addLocalForceAtLocalPos(physx_PxRigidBody* body_pod, physx_PxVec3 const* force_pod, physx_PxVec3 const* pos_pod, PxForceMode mode_pod, bool wakeup) {
        physx::PxRigidBody& body = reinterpret_cast<physx::PxRigidBody&>(*body_pod);
        physx::PxVec3 const& force = reinterpret_cast<physx::PxVec3 const&>(*force_pod);
        physx::PxVec3 const& pos = reinterpret_cast<physx::PxVec3 const&>(*pos_pod);
        auto mode = static_cast<physx::PxForceMode::Enum>(mode_pod);
        PxRigidBodyExt::addLocalForceAtLocalPos(body, force, pos, mode, wakeup);
    }

    physx_PxVec3 PxRigidBodyExt_getVelocityAtPos(physx_PxRigidBody const* body_pod, physx_PxVec3 const* pos_pod) {
        physx::PxRigidBody const& body = reinterpret_cast<physx::PxRigidBody const&>(*body_pod);
        physx::PxVec3 const& pos = reinterpret_cast<physx::PxVec3 const&>(*pos_pod);
        physx::PxVec3 return_val = PxRigidBodyExt::getVelocityAtPos(body, pos);
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3 PxRigidBodyExt_getLocalVelocityAtLocalPos(physx_PxRigidBody const* body_pod, physx_PxVec3 const* pos_pod) {
        physx::PxRigidBody const& body = reinterpret_cast<physx::PxRigidBody const&>(*body_pod);
        physx::PxVec3 const& pos = reinterpret_cast<physx::PxVec3 const&>(*pos_pod);
        physx::PxVec3 return_val = PxRigidBodyExt::getLocalVelocityAtLocalPos(body, pos);
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxVec3 PxRigidBodyExt_getVelocityAtOffset(physx_PxRigidBody const* body_pod, physx_PxVec3 const* pos_pod) {
        physx::PxRigidBody const& body = reinterpret_cast<physx::PxRigidBody const&>(*body_pod);
        physx::PxVec3 const& pos = reinterpret_cast<physx::PxVec3 const&>(*pos_pod);
        physx::PxVec3 return_val = PxRigidBodyExt::getVelocityAtOffset(body, pos);
        physx_PxVec3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxRigidBodyExt_computeVelocityDeltaFromImpulse(physx_PxRigidBody const* body_pod, physx_PxVec3 const* impulsiveForce_pod, physx_PxVec3 const* impulsiveTorque_pod, physx_PxVec3* deltaLinearVelocity_pod, physx_PxVec3* deltaAngularVelocity_pod) {
        physx::PxRigidBody const& body = reinterpret_cast<physx::PxRigidBody const&>(*body_pod);
        physx::PxVec3 const& impulsiveForce = reinterpret_cast<physx::PxVec3 const&>(*impulsiveForce_pod);
        physx::PxVec3 const& impulsiveTorque = reinterpret_cast<physx::PxVec3 const&>(*impulsiveTorque_pod);
        physx::PxVec3& deltaLinearVelocity = reinterpret_cast<physx::PxVec3&>(*deltaLinearVelocity_pod);
        physx::PxVec3& deltaAngularVelocity = reinterpret_cast<physx::PxVec3&>(*deltaAngularVelocity_pod);
        PxRigidBodyExt::computeVelocityDeltaFromImpulse(body, impulsiveForce, impulsiveTorque, deltaLinearVelocity, deltaAngularVelocity);
    }

    void PxRigidBodyExt_computeVelocityDeltaFromImpulse_1(physx_PxRigidBody const* body_pod, physx_PxTransform const* globalPose_pod, physx_PxVec3 const* point_pod, physx_PxVec3 const* impulse_pod, float invMassScale, float invInertiaScale, physx_PxVec3* deltaLinearVelocity_pod, physx_PxVec3* deltaAngularVelocity_pod) {
        physx::PxRigidBody const& body = reinterpret_cast<physx::PxRigidBody const&>(*body_pod);
        physx::PxTransform const& globalPose = reinterpret_cast<physx::PxTransform const&>(*globalPose_pod);
        physx::PxVec3 const& point = reinterpret_cast<physx::PxVec3 const&>(*point_pod);
        physx::PxVec3 const& impulse = reinterpret_cast<physx::PxVec3 const&>(*impulse_pod);
        physx::PxVec3& deltaLinearVelocity = reinterpret_cast<physx::PxVec3&>(*deltaLinearVelocity_pod);
        physx::PxVec3& deltaAngularVelocity = reinterpret_cast<physx::PxVec3&>(*deltaAngularVelocity_pod);
        PxRigidBodyExt::computeVelocityDeltaFromImpulse(body, globalPose, point, impulse, invMassScale, invInertiaScale, deltaLinearVelocity, deltaAngularVelocity);
    }

    void PxRigidBodyExt_computeLinearAngularImpulse(physx_PxRigidBody const* body_pod, physx_PxTransform const* globalPose_pod, physx_PxVec3 const* point_pod, physx_PxVec3 const* impulse_pod, float invMassScale, float invInertiaScale, physx_PxVec3* linearImpulse_pod, physx_PxVec3* angularImpulse_pod) {
        physx::PxRigidBody const& body = reinterpret_cast<physx::PxRigidBody const&>(*body_pod);
        physx::PxTransform const& globalPose = reinterpret_cast<physx::PxTransform const&>(*globalPose_pod);
        physx::PxVec3 const& point = reinterpret_cast<physx::PxVec3 const&>(*point_pod);
        physx::PxVec3 const& impulse = reinterpret_cast<physx::PxVec3 const&>(*impulse_pod);
        physx::PxVec3& linearImpulse = reinterpret_cast<physx::PxVec3&>(*linearImpulse_pod);
        physx::PxVec3& angularImpulse = reinterpret_cast<physx::PxVec3&>(*angularImpulse_pod);
        PxRigidBodyExt::computeLinearAngularImpulse(body, globalPose, point, impulse, invMassScale, invInertiaScale, linearImpulse, angularImpulse);
    }

    bool PxRigidBodyExt_linearSweepSingle(physx_PxRigidBody* body_pod, physx_PxScene* scene_pod, physx_PxVec3 const* unitDir_pod, float distance, PxHitFlags outputFlags_pod, physx_PxSweepHit* closestHit_pod, uint32_t* shapeIndex_pod, physx_PxQueryFilterData const* filterData_pod, physx_PxQueryFilterCallback* filterCall_pod, physx_PxQueryCache const* cache_pod, float inflation) {
        physx::PxRigidBody& body = reinterpret_cast<physx::PxRigidBody&>(*body_pod);
        physx::PxScene& scene = reinterpret_cast<physx::PxScene&>(*scene_pod);
        physx::PxVec3 const& unitDir = reinterpret_cast<physx::PxVec3 const&>(*unitDir_pod);
        auto outputFlags = physx::PxHitFlags(outputFlags_pod);
        physx::PxSweepHit& closestHit = reinterpret_cast<physx::PxSweepHit&>(*closestHit_pod);
        uint32_t& shapeIndex = *shapeIndex_pod;
        physx::PxQueryFilterData const& filterData = reinterpret_cast<physx::PxQueryFilterData const&>(*filterData_pod);
        physx::PxQueryFilterCallback* filterCall = reinterpret_cast<physx::PxQueryFilterCallback*>(filterCall_pod);
        physx::PxQueryCache const* cache = reinterpret_cast<physx::PxQueryCache const*>(cache_pod);
        bool return_val = PxRigidBodyExt::linearSweepSingle(body, scene, unitDir, distance, outputFlags, closestHit, shapeIndex, filterData, filterCall, cache, inflation);
        return return_val;
    }

    uint32_t PxRigidBodyExt_linearSweepMultiple(physx_PxRigidBody* body_pod, physx_PxScene* scene_pod, physx_PxVec3 const* unitDir_pod, float distance, PxHitFlags outputFlags_pod, physx_PxSweepHit* touchHitBuffer_pod, uint32_t* touchHitShapeIndices, uint32_t touchHitBufferSize, physx_PxSweepHit* block_pod, int32_t* blockingShapeIndex_pod, bool* overflow_pod, physx_PxQueryFilterData const* filterData_pod, physx_PxQueryFilterCallback* filterCall_pod, physx_PxQueryCache const* cache_pod, float inflation) {
        physx::PxRigidBody& body = reinterpret_cast<physx::PxRigidBody&>(*body_pod);
        physx::PxScene& scene = reinterpret_cast<physx::PxScene&>(*scene_pod);
        physx::PxVec3 const& unitDir = reinterpret_cast<physx::PxVec3 const&>(*unitDir_pod);
        auto outputFlags = physx::PxHitFlags(outputFlags_pod);
        physx::PxSweepHit* touchHitBuffer = reinterpret_cast<physx::PxSweepHit*>(touchHitBuffer_pod);
        physx::PxSweepHit& block = reinterpret_cast<physx::PxSweepHit&>(*block_pod);
        int32_t& blockingShapeIndex = *blockingShapeIndex_pod;
        bool& overflow = *overflow_pod;
        physx::PxQueryFilterData const& filterData = reinterpret_cast<physx::PxQueryFilterData const&>(*filterData_pod);
        physx::PxQueryFilterCallback* filterCall = reinterpret_cast<physx::PxQueryFilterCallback*>(filterCall_pod);
        physx::PxQueryCache const* cache = reinterpret_cast<physx::PxQueryCache const*>(cache_pod);
        uint32_t return_val = PxRigidBodyExt::linearSweepMultiple(body, scene, unitDir, distance, outputFlags, touchHitBuffer, touchHitShapeIndices, touchHitBufferSize, block, blockingShapeIndex, overflow, filterData, filterCall, cache, inflation);
        return return_val;
    }

    physx_PxTransform PxShapeExt_getGlobalPose(physx_PxShape const* shape_pod, physx_PxRigidActor const* actor_pod) {
        physx::PxShape const& shape = reinterpret_cast<physx::PxShape const&>(*shape_pod);
        physx::PxRigidActor const& actor = reinterpret_cast<physx::PxRigidActor const&>(*actor_pod);
        physx::PxTransform return_val = PxShapeExt::getGlobalPose(shape, actor);
        physx_PxTransform return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    uint32_t PxShapeExt_raycast(physx_PxShape const* shape_pod, physx_PxRigidActor const* actor_pod, physx_PxVec3 const* rayOrigin_pod, physx_PxVec3 const* rayDir_pod, float maxDist, PxHitFlags hitFlags_pod, uint32_t maxHits, physx_PxRaycastHit* rayHits_pod) {
        physx::PxShape const& shape = reinterpret_cast<physx::PxShape const&>(*shape_pod);
        physx::PxRigidActor const& actor = reinterpret_cast<physx::PxRigidActor const&>(*actor_pod);
        physx::PxVec3 const& rayOrigin = reinterpret_cast<physx::PxVec3 const&>(*rayOrigin_pod);
        physx::PxVec3 const& rayDir = reinterpret_cast<physx::PxVec3 const&>(*rayDir_pod);
        auto hitFlags = physx::PxHitFlags(hitFlags_pod);
        physx::PxRaycastHit* rayHits = reinterpret_cast<physx::PxRaycastHit*>(rayHits_pod);
        uint32_t return_val = PxShapeExt::raycast(shape, actor, rayOrigin, rayDir, maxDist, hitFlags, maxHits, rayHits);
        return return_val;
    }

    bool PxShapeExt_overlap(physx_PxShape const* shape_pod, physx_PxRigidActor const* actor_pod, physx_PxGeometry const* otherGeom_pod, physx_PxTransform const* otherGeomPose_pod) {
        physx::PxShape const& shape = reinterpret_cast<physx::PxShape const&>(*shape_pod);
        physx::PxRigidActor const& actor = reinterpret_cast<physx::PxRigidActor const&>(*actor_pod);
        physx::PxGeometry const& otherGeom = reinterpret_cast<physx::PxGeometry const&>(*otherGeom_pod);
        physx::PxTransform const& otherGeomPose = reinterpret_cast<physx::PxTransform const&>(*otherGeomPose_pod);
        bool return_val = PxShapeExt::overlap(shape, actor, otherGeom, otherGeomPose);
        return return_val;
    }

    bool PxShapeExt_sweep(physx_PxShape const* shape_pod, physx_PxRigidActor const* actor_pod, physx_PxVec3 const* unitDir_pod, float distance, physx_PxGeometry const* otherGeom_pod, physx_PxTransform const* otherGeomPose_pod, physx_PxSweepHit* sweepHit_pod, PxHitFlags hitFlags_pod) {
        physx::PxShape const& shape = reinterpret_cast<physx::PxShape const&>(*shape_pod);
        physx::PxRigidActor const& actor = reinterpret_cast<physx::PxRigidActor const&>(*actor_pod);
        physx::PxVec3 const& unitDir = reinterpret_cast<physx::PxVec3 const&>(*unitDir_pod);
        physx::PxGeometry const& otherGeom = reinterpret_cast<physx::PxGeometry const&>(*otherGeom_pod);
        physx::PxTransform const& otherGeomPose = reinterpret_cast<physx::PxTransform const&>(*otherGeomPose_pod);
        physx::PxSweepHit& sweepHit = reinterpret_cast<physx::PxSweepHit&>(*sweepHit_pod);
        auto hitFlags = physx::PxHitFlags(hitFlags_pod);
        bool return_val = PxShapeExt::sweep(shape, actor, unitDir, distance, otherGeom, otherGeomPose, sweepHit, hitFlags);
        return return_val;
    }

    physx_PxBounds3 PxShapeExt_getWorldBounds(physx_PxShape const* shape_pod, physx_PxRigidActor const* actor_pod, float inflation) {
        physx::PxShape const& shape = reinterpret_cast<physx::PxShape const&>(*shape_pod);
        physx::PxRigidActor const& actor = reinterpret_cast<physx::PxRigidActor const&>(*actor_pod);
        physx::PxBounds3 return_val = PxShapeExt::getWorldBounds(shape, actor, inflation);
        physx_PxBounds3 return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxMeshOverlapUtil* PxMeshOverlapUtil_new_alloc() {
        auto return_val = new physx::PxMeshOverlapUtil();
        auto return_val_pod = reinterpret_cast<physx_PxMeshOverlapUtil*>(return_val);
        return return_val_pod;
    }

    void PxMeshOverlapUtil_delete(physx_PxMeshOverlapUtil* self__pod) {
        physx::PxMeshOverlapUtil* self_ = reinterpret_cast<physx::PxMeshOverlapUtil*>(self__pod);
        delete self_;
    }

    uint32_t PxMeshOverlapUtil_findOverlap(physx_PxMeshOverlapUtil* self__pod, physx_PxGeometry const* geom_pod, physx_PxTransform const* geomPose_pod, physx_PxTriangleMeshGeometry const* meshGeom_pod, physx_PxTransform const* meshPose_pod) {
        physx::PxMeshOverlapUtil* self_ = reinterpret_cast<physx::PxMeshOverlapUtil*>(self__pod);
        physx::PxGeometry const& geom = reinterpret_cast<physx::PxGeometry const&>(*geom_pod);
        physx::PxTransform const& geomPose = reinterpret_cast<physx::PxTransform const&>(*geomPose_pod);
        physx::PxTriangleMeshGeometry const& meshGeom = reinterpret_cast<physx::PxTriangleMeshGeometry const&>(*meshGeom_pod);
        physx::PxTransform const& meshPose = reinterpret_cast<physx::PxTransform const&>(*meshPose_pod);
        uint32_t return_val = self_->findOverlap(geom, geomPose, meshGeom, meshPose);
        return return_val;
    }

    uint32_t PxMeshOverlapUtil_findOverlap_1(physx_PxMeshOverlapUtil* self__pod, physx_PxGeometry const* geom_pod, physx_PxTransform const* geomPose_pod, physx_PxHeightFieldGeometry const* hfGeom_pod, physx_PxTransform const* hfPose_pod) {
        physx::PxMeshOverlapUtil* self_ = reinterpret_cast<physx::PxMeshOverlapUtil*>(self__pod);
        physx::PxGeometry const& geom = reinterpret_cast<physx::PxGeometry const&>(*geom_pod);
        physx::PxTransform const& geomPose = reinterpret_cast<physx::PxTransform const&>(*geomPose_pod);
        physx::PxHeightFieldGeometry const& hfGeom = reinterpret_cast<physx::PxHeightFieldGeometry const&>(*hfGeom_pod);
        physx::PxTransform const& hfPose = reinterpret_cast<physx::PxTransform const&>(*hfPose_pod);
        uint32_t return_val = self_->findOverlap(geom, geomPose, hfGeom, hfPose);
        return return_val;
    }

    uint32_t const* PxMeshOverlapUtil_getResults(physx_PxMeshOverlapUtil const* self__pod) {
        physx::PxMeshOverlapUtil const* self_ = reinterpret_cast<physx::PxMeshOverlapUtil const*>(self__pod);
        uint32_t const* return_val = self_->getResults();
        return return_val;
    }

    uint32_t PxMeshOverlapUtil_getNbResults(physx_PxMeshOverlapUtil const* self__pod) {
        physx::PxMeshOverlapUtil const* self_ = reinterpret_cast<physx::PxMeshOverlapUtil const*>(self__pod);
        uint32_t return_val = self_->getNbResults();
        return return_val;
    }

    bool phys_PxComputeTriangleMeshPenetration(physx_PxVec3* direction_pod, float* depth_pod, physx_PxGeometry const* geom_pod, physx_PxTransform const* geomPose_pod, physx_PxTriangleMeshGeometry const* meshGeom_pod, physx_PxTransform const* meshPose_pod, uint32_t maxIter, uint32_t* usedIter) {
        physx::PxVec3& direction = reinterpret_cast<physx::PxVec3&>(*direction_pod);
        float& depth = *depth_pod;
        physx::PxGeometry const& geom = reinterpret_cast<physx::PxGeometry const&>(*geom_pod);
        physx::PxTransform const& geomPose = reinterpret_cast<physx::PxTransform const&>(*geomPose_pod);
        physx::PxTriangleMeshGeometry const& meshGeom = reinterpret_cast<physx::PxTriangleMeshGeometry const&>(*meshGeom_pod);
        physx::PxTransform const& meshPose = reinterpret_cast<physx::PxTransform const&>(*meshPose_pod);
        bool return_val = PxComputeTriangleMeshPenetration(direction, depth, geom, geomPose, meshGeom, meshPose, maxIter, usedIter);
        return return_val;
    }

    bool phys_PxComputeHeightFieldPenetration(physx_PxVec3* direction_pod, float* depth_pod, physx_PxGeometry const* geom_pod, physx_PxTransform const* geomPose_pod, physx_PxHeightFieldGeometry const* heightFieldGeom_pod, physx_PxTransform const* heightFieldPose_pod, uint32_t maxIter, uint32_t* usedIter) {
        physx::PxVec3& direction = reinterpret_cast<physx::PxVec3&>(*direction_pod);
        float& depth = *depth_pod;
        physx::PxGeometry const& geom = reinterpret_cast<physx::PxGeometry const&>(*geom_pod);
        physx::PxTransform const& geomPose = reinterpret_cast<physx::PxTransform const&>(*geomPose_pod);
        physx::PxHeightFieldGeometry const& heightFieldGeom = reinterpret_cast<physx::PxHeightFieldGeometry const&>(*heightFieldGeom_pod);
        physx::PxTransform const& heightFieldPose = reinterpret_cast<physx::PxTransform const&>(*heightFieldPose_pod);
        bool return_val = PxComputeHeightFieldPenetration(direction, depth, geom, geomPose, heightFieldGeom, heightFieldPose, maxIter, usedIter);
        return return_val;
    }

    physx_PxXmlMiscParameter PxXmlMiscParameter_new() {
        PxXmlMiscParameter return_val;
        physx_PxXmlMiscParameter return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    physx_PxXmlMiscParameter PxXmlMiscParameter_new_1(physx_PxVec3* inUpVector_pod, physx_PxTolerancesScale inScale_pod) {
        physx::PxVec3& inUpVector = reinterpret_cast<physx::PxVec3&>(*inUpVector_pod);
        physx::PxTolerancesScale inScale;
        memcpy(&inScale, &inScale_pod, sizeof(inScale));
        PxXmlMiscParameter return_val(inUpVector, inScale);
        physx_PxXmlMiscParameter return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxSerialization_isSerializable(physx_PxCollection* collection_pod, physx_PxSerializationRegistry* sr_pod, physx_PxCollection const* externalReferences_pod) {
        physx::PxCollection& collection = reinterpret_cast<physx::PxCollection&>(*collection_pod);
        physx::PxSerializationRegistry& sr = reinterpret_cast<physx::PxSerializationRegistry&>(*sr_pod);
        physx::PxCollection const* externalReferences = reinterpret_cast<physx::PxCollection const*>(externalReferences_pod);
        bool return_val = PxSerialization::isSerializable(collection, sr, externalReferences);
        return return_val;
    }

    void PxSerialization_complete(physx_PxCollection* collection_pod, physx_PxSerializationRegistry* sr_pod, physx_PxCollection const* exceptFor_pod, bool followJoints) {
        physx::PxCollection& collection = reinterpret_cast<physx::PxCollection&>(*collection_pod);
        physx::PxSerializationRegistry& sr = reinterpret_cast<physx::PxSerializationRegistry&>(*sr_pod);
        physx::PxCollection const* exceptFor = reinterpret_cast<physx::PxCollection const*>(exceptFor_pod);
        PxSerialization::complete(collection, sr, exceptFor, followJoints);
    }

    void PxSerialization_createSerialObjectIds(physx_PxCollection* collection_pod, uint64_t base) {
        physx::PxCollection& collection = reinterpret_cast<physx::PxCollection&>(*collection_pod);
        PxSerialization::createSerialObjectIds(collection, base);
    }

    physx_PxCollection* PxSerialization_createCollectionFromXml(physx_PxInputData* inputData_pod, physx_PxCooking* cooking_pod, physx_PxSerializationRegistry* sr_pod, physx_PxCollection const* externalRefs_pod, physx_PxStringTable* stringTable_pod, physx_PxXmlMiscParameter* outArgs_pod) {
        physx::PxInputData& inputData = reinterpret_cast<physx::PxInputData&>(*inputData_pod);
        physx::PxCooking& cooking = reinterpret_cast<physx::PxCooking&>(*cooking_pod);
        physx::PxSerializationRegistry& sr = reinterpret_cast<physx::PxSerializationRegistry&>(*sr_pod);
        physx::PxCollection const* externalRefs = reinterpret_cast<physx::PxCollection const*>(externalRefs_pod);
        physx::PxStringTable* stringTable = reinterpret_cast<physx::PxStringTable*>(stringTable_pod);
        physx::PxXmlMiscParameter* outArgs = reinterpret_cast<physx::PxXmlMiscParameter*>(outArgs_pod);
        physx::PxCollection* return_val = PxSerialization::createCollectionFromXml(inputData, cooking, sr, externalRefs, stringTable, outArgs);
        auto return_val_pod = reinterpret_cast<physx_PxCollection*>(return_val);
        return return_val_pod;
    }

    physx_PxCollection* PxSerialization_createCollectionFromBinary(void* memBlock, physx_PxSerializationRegistry* sr_pod, physx_PxCollection const* externalRefs_pod) {
        physx::PxSerializationRegistry& sr = reinterpret_cast<physx::PxSerializationRegistry&>(*sr_pod);
        physx::PxCollection const* externalRefs = reinterpret_cast<physx::PxCollection const*>(externalRefs_pod);
        physx::PxCollection* return_val = PxSerialization::createCollectionFromBinary(memBlock, sr, externalRefs);
        auto return_val_pod = reinterpret_cast<physx_PxCollection*>(return_val);
        return return_val_pod;
    }

    bool PxSerialization_serializeCollectionToXml(physx_PxOutputStream* outputStream_pod, physx_PxCollection* collection_pod, physx_PxSerializationRegistry* sr_pod, physx_PxCooking* cooking_pod, physx_PxCollection const* externalRefs_pod, physx_PxXmlMiscParameter* inArgs_pod) {
        physx::PxOutputStream& outputStream = reinterpret_cast<physx::PxOutputStream&>(*outputStream_pod);
        physx::PxCollection& collection = reinterpret_cast<physx::PxCollection&>(*collection_pod);
        physx::PxSerializationRegistry& sr = reinterpret_cast<physx::PxSerializationRegistry&>(*sr_pod);
        physx::PxCooking* cooking = reinterpret_cast<physx::PxCooking*>(cooking_pod);
        physx::PxCollection const* externalRefs = reinterpret_cast<physx::PxCollection const*>(externalRefs_pod);
        physx::PxXmlMiscParameter* inArgs = reinterpret_cast<physx::PxXmlMiscParameter*>(inArgs_pod);
        bool return_val = PxSerialization::serializeCollectionToXml(outputStream, collection, sr, cooking, externalRefs, inArgs);
        return return_val;
    }

    bool PxSerialization_serializeCollectionToBinary(physx_PxOutputStream* outputStream_pod, physx_PxCollection* collection_pod, physx_PxSerializationRegistry* sr_pod, physx_PxCollection const* externalRefs_pod, bool exportNames) {
        physx::PxOutputStream& outputStream = reinterpret_cast<physx::PxOutputStream&>(*outputStream_pod);
        physx::PxCollection& collection = reinterpret_cast<physx::PxCollection&>(*collection_pod);
        physx::PxSerializationRegistry& sr = reinterpret_cast<physx::PxSerializationRegistry&>(*sr_pod);
        physx::PxCollection const* externalRefs = reinterpret_cast<physx::PxCollection const*>(externalRefs_pod);
        bool return_val = PxSerialization::serializeCollectionToBinary(outputStream, collection, sr, externalRefs, exportNames);
        return return_val;
    }

    physx_PxSerializationRegistry* PxSerialization_createSerializationRegistry(physx_PxPhysics* physics_pod) {
        physx::PxPhysics& physics = reinterpret_cast<physx::PxPhysics&>(*physics_pod);
        physx::PxSerializationRegistry* return_val = PxSerialization::createSerializationRegistry(physics);
        auto return_val_pod = reinterpret_cast<physx_PxSerializationRegistry*>(return_val);
        return return_val_pod;
    }

    void PxDefaultCpuDispatcher_release(physx_PxDefaultCpuDispatcher* self__pod) {
        physx::PxDefaultCpuDispatcher* self_ = reinterpret_cast<physx::PxDefaultCpuDispatcher*>(self__pod);
        self_->release();
    }

    void PxDefaultCpuDispatcher_setRunProfiled(physx_PxDefaultCpuDispatcher* self__pod, bool runProfiled) {
        physx::PxDefaultCpuDispatcher* self_ = reinterpret_cast<physx::PxDefaultCpuDispatcher*>(self__pod);
        self_->setRunProfiled(runProfiled);
    }

    bool PxDefaultCpuDispatcher_getRunProfiled(physx_PxDefaultCpuDispatcher const* self__pod) {
        physx::PxDefaultCpuDispatcher const* self_ = reinterpret_cast<physx::PxDefaultCpuDispatcher const*>(self__pod);
        bool return_val = self_->getRunProfiled();
        return return_val;
    }

    physx_PxDefaultCpuDispatcher* phys_PxDefaultCpuDispatcherCreate(uint32_t numThreads, uint32_t* affinityMasks, PxDefaultCpuDispatcherWaitForWorkMode mode_pod, uint32_t yieldProcessorCount) {
        auto mode = static_cast<physx::PxDefaultCpuDispatcherWaitForWorkMode::Enum>(mode_pod);
        physx::PxDefaultCpuDispatcher* return_val = PxDefaultCpuDispatcherCreate(numThreads, affinityMasks, mode, yieldProcessorCount);
        auto return_val_pod = reinterpret_cast<physx_PxDefaultCpuDispatcher*>(return_val);
        return return_val_pod;
    }

    bool phys_PxBuildSmoothNormals(uint32_t nbTris, uint32_t nbVerts, physx_PxVec3 const* verts_pod, uint32_t const* dFaces, uint16_t const* wFaces, physx_PxVec3* normals_pod, bool flip) {
        physx::PxVec3 const* verts = reinterpret_cast<physx::PxVec3 const*>(verts_pod);
        physx::PxVec3* normals = reinterpret_cast<physx::PxVec3*>(normals_pod);
        bool return_val = PxBuildSmoothNormals(nbTris, nbVerts, verts, dFaces, wFaces, normals, flip);
        return return_val;
    }

    physx_PxRigidDynamic* phys_PxCreateDynamic(physx_PxPhysics* sdk_pod, physx_PxTransform const* transform_pod, physx_PxGeometry const* geometry_pod, physx_PxMaterial* material_pod, float density, physx_PxTransform const* shapeOffset_pod) {
        physx::PxPhysics& sdk = reinterpret_cast<physx::PxPhysics&>(*sdk_pod);
        physx::PxTransform const& transform = reinterpret_cast<physx::PxTransform const&>(*transform_pod);
        physx::PxGeometry const& geometry = reinterpret_cast<physx::PxGeometry const&>(*geometry_pod);
        physx::PxMaterial& material = reinterpret_cast<physx::PxMaterial&>(*material_pod);
        physx::PxTransform const& shapeOffset = reinterpret_cast<physx::PxTransform const&>(*shapeOffset_pod);
        physx::PxRigidDynamic* return_val = PxCreateDynamic(sdk, transform, geometry, material, density, shapeOffset);
        auto return_val_pod = reinterpret_cast<physx_PxRigidDynamic*>(return_val);
        return return_val_pod;
    }

    physx_PxRigidDynamic* phys_PxCreateDynamic_1(physx_PxPhysics* sdk_pod, physx_PxTransform const* transform_pod, physx_PxShape* shape_pod, float density) {
        physx::PxPhysics& sdk = reinterpret_cast<physx::PxPhysics&>(*sdk_pod);
        physx::PxTransform const& transform = reinterpret_cast<physx::PxTransform const&>(*transform_pod);
        physx::PxShape& shape = reinterpret_cast<physx::PxShape&>(*shape_pod);
        physx::PxRigidDynamic* return_val = PxCreateDynamic(sdk, transform, shape, density);
        auto return_val_pod = reinterpret_cast<physx_PxRigidDynamic*>(return_val);
        return return_val_pod;
    }

    physx_PxRigidDynamic* phys_PxCreateKinematic(physx_PxPhysics* sdk_pod, physx_PxTransform const* transform_pod, physx_PxGeometry const* geometry_pod, physx_PxMaterial* material_pod, float density, physx_PxTransform const* shapeOffset_pod) {
        physx::PxPhysics& sdk = reinterpret_cast<physx::PxPhysics&>(*sdk_pod);
        physx::PxTransform const& transform = reinterpret_cast<physx::PxTransform const&>(*transform_pod);
        physx::PxGeometry const& geometry = reinterpret_cast<physx::PxGeometry const&>(*geometry_pod);
        physx::PxMaterial& material = reinterpret_cast<physx::PxMaterial&>(*material_pod);
        physx::PxTransform const& shapeOffset = reinterpret_cast<physx::PxTransform const&>(*shapeOffset_pod);
        physx::PxRigidDynamic* return_val = PxCreateKinematic(sdk, transform, geometry, material, density, shapeOffset);
        auto return_val_pod = reinterpret_cast<physx_PxRigidDynamic*>(return_val);
        return return_val_pod;
    }

    physx_PxRigidDynamic* phys_PxCreateKinematic_1(physx_PxPhysics* sdk_pod, physx_PxTransform const* transform_pod, physx_PxShape* shape_pod, float density) {
        physx::PxPhysics& sdk = reinterpret_cast<physx::PxPhysics&>(*sdk_pod);
        physx::PxTransform const& transform = reinterpret_cast<physx::PxTransform const&>(*transform_pod);
        physx::PxShape& shape = reinterpret_cast<physx::PxShape&>(*shape_pod);
        physx::PxRigidDynamic* return_val = PxCreateKinematic(sdk, transform, shape, density);
        auto return_val_pod = reinterpret_cast<physx_PxRigidDynamic*>(return_val);
        return return_val_pod;
    }

    physx_PxRigidStatic* phys_PxCreateStatic(physx_PxPhysics* sdk_pod, physx_PxTransform const* transform_pod, physx_PxGeometry const* geometry_pod, physx_PxMaterial* material_pod, physx_PxTransform const* shapeOffset_pod) {
        physx::PxPhysics& sdk = reinterpret_cast<physx::PxPhysics&>(*sdk_pod);
        physx::PxTransform const& transform = reinterpret_cast<physx::PxTransform const&>(*transform_pod);
        physx::PxGeometry const& geometry = reinterpret_cast<physx::PxGeometry const&>(*geometry_pod);
        physx::PxMaterial& material = reinterpret_cast<physx::PxMaterial&>(*material_pod);
        physx::PxTransform const& shapeOffset = reinterpret_cast<physx::PxTransform const&>(*shapeOffset_pod);
        physx::PxRigidStatic* return_val = PxCreateStatic(sdk, transform, geometry, material, shapeOffset);
        auto return_val_pod = reinterpret_cast<physx_PxRigidStatic*>(return_val);
        return return_val_pod;
    }

    physx_PxRigidStatic* phys_PxCreateStatic_1(physx_PxPhysics* sdk_pod, physx_PxTransform const* transform_pod, physx_PxShape* shape_pod) {
        physx::PxPhysics& sdk = reinterpret_cast<physx::PxPhysics&>(*sdk_pod);
        physx::PxTransform const& transform = reinterpret_cast<physx::PxTransform const&>(*transform_pod);
        physx::PxShape& shape = reinterpret_cast<physx::PxShape&>(*shape_pod);
        physx::PxRigidStatic* return_val = PxCreateStatic(sdk, transform, shape);
        auto return_val_pod = reinterpret_cast<physx_PxRigidStatic*>(return_val);
        return return_val_pod;
    }

    physx_PxShape* phys_PxCloneShape(physx_PxPhysics* physicsSDK_pod, physx_PxShape const* shape_pod, bool isExclusive) {
        physx::PxPhysics& physicsSDK = reinterpret_cast<physx::PxPhysics&>(*physicsSDK_pod);
        physx::PxShape const& shape = reinterpret_cast<physx::PxShape const&>(*shape_pod);
        physx::PxShape* return_val = PxCloneShape(physicsSDK, shape, isExclusive);
        auto return_val_pod = reinterpret_cast<physx_PxShape*>(return_val);
        return return_val_pod;
    }

    physx_PxRigidStatic* phys_PxCloneStatic(physx_PxPhysics* physicsSDK_pod, physx_PxTransform const* transform_pod, physx_PxRigidActor const* actor_pod) {
        physx::PxPhysics& physicsSDK = reinterpret_cast<physx::PxPhysics&>(*physicsSDK_pod);
        physx::PxTransform const& transform = reinterpret_cast<physx::PxTransform const&>(*transform_pod);
        physx::PxRigidActor const& actor = reinterpret_cast<physx::PxRigidActor const&>(*actor_pod);
        physx::PxRigidStatic* return_val = PxCloneStatic(physicsSDK, transform, actor);
        auto return_val_pod = reinterpret_cast<physx_PxRigidStatic*>(return_val);
        return return_val_pod;
    }

    physx_PxRigidDynamic* phys_PxCloneDynamic(physx_PxPhysics* physicsSDK_pod, physx_PxTransform const* transform_pod, physx_PxRigidDynamic const* body_pod) {
        physx::PxPhysics& physicsSDK = reinterpret_cast<physx::PxPhysics&>(*physicsSDK_pod);
        physx::PxTransform const& transform = reinterpret_cast<physx::PxTransform const&>(*transform_pod);
        physx::PxRigidDynamic const& body = reinterpret_cast<physx::PxRigidDynamic const&>(*body_pod);
        physx::PxRigidDynamic* return_val = PxCloneDynamic(physicsSDK, transform, body);
        auto return_val_pod = reinterpret_cast<physx_PxRigidDynamic*>(return_val);
        return return_val_pod;
    }

    physx_PxRigidStatic* phys_PxCreatePlane(physx_PxPhysics* sdk_pod, physx_PxPlane const* plane_pod, physx_PxMaterial* material_pod) {
        physx::PxPhysics& sdk = reinterpret_cast<physx::PxPhysics&>(*sdk_pod);
        physx::PxPlane const& plane = reinterpret_cast<physx::PxPlane const&>(*plane_pod);
        physx::PxMaterial& material = reinterpret_cast<physx::PxMaterial&>(*material_pod);
        physx::PxRigidStatic* return_val = PxCreatePlane(sdk, plane, material);
        auto return_val_pod = reinterpret_cast<physx_PxRigidStatic*>(return_val);
        return return_val_pod;
    }

    void phys_PxScaleRigidActor(physx_PxRigidActor* actor_pod, float scale, bool scaleMassProps) {
        physx::PxRigidActor& actor = reinterpret_cast<physx::PxRigidActor&>(*actor_pod);
        PxScaleRigidActor(actor, scale, scaleMassProps);
    }

    physx_PxStringTable* PxStringTableExt_createStringTable(physx_PxAllocatorCallback* inAllocator_pod) {
        physx::PxAllocatorCallback& inAllocator = reinterpret_cast<physx::PxAllocatorCallback&>(*inAllocator_pod);
        physx::PxStringTable& return_val = PxStringTableExt::createStringTable(inAllocator);
        auto return_val_pod = reinterpret_cast<physx_PxStringTable*>(&return_val);
        return return_val_pod;
    }

    uint32_t PxBroadPhaseExt_createRegionsFromWorldBounds(physx_PxBounds3* regions_pod, physx_PxBounds3 const* globalBounds_pod, uint32_t nbSubdiv, uint32_t upAxis) {
        physx::PxBounds3* regions = reinterpret_cast<physx::PxBounds3*>(regions_pod);
        physx::PxBounds3 const& globalBounds = reinterpret_cast<physx::PxBounds3 const&>(*globalBounds_pod);
        uint32_t return_val = PxBroadPhaseExt::createRegionsFromWorldBounds(regions, globalBounds, nbSubdiv, upAxis);
        return return_val;
    }

    bool PxSceneQueryExt_raycastAny(physx_PxScene const* scene_pod, physx_PxVec3 const* origin_pod, physx_PxVec3 const* unitDir_pod, float distance, physx_PxQueryHit* hit_pod, physx_PxQueryFilterData const* filterData_pod, physx_PxQueryFilterCallback* filterCall_pod, physx_PxQueryCache const* cache_pod) {
        physx::PxScene const& scene = reinterpret_cast<physx::PxScene const&>(*scene_pod);
        physx::PxVec3 const& origin = reinterpret_cast<physx::PxVec3 const&>(*origin_pod);
        physx::PxVec3 const& unitDir = reinterpret_cast<physx::PxVec3 const&>(*unitDir_pod);
        physx::PxQueryHit& hit = reinterpret_cast<physx::PxQueryHit&>(*hit_pod);
        physx::PxQueryFilterData const& filterData = reinterpret_cast<physx::PxQueryFilterData const&>(*filterData_pod);
        physx::PxQueryFilterCallback* filterCall = reinterpret_cast<physx::PxQueryFilterCallback*>(filterCall_pod);
        physx::PxQueryCache const* cache = reinterpret_cast<physx::PxQueryCache const*>(cache_pod);
        bool return_val = PxSceneQueryExt::raycastAny(scene, origin, unitDir, distance, hit, filterData, filterCall, cache);
        return return_val;
    }

    bool PxSceneQueryExt_raycastSingle(physx_PxScene const* scene_pod, physx_PxVec3 const* origin_pod, physx_PxVec3 const* unitDir_pod, float distance, PxHitFlags outputFlags_pod, physx_PxRaycastHit* hit_pod, physx_PxQueryFilterData const* filterData_pod, physx_PxQueryFilterCallback* filterCall_pod, physx_PxQueryCache const* cache_pod) {
        physx::PxScene const& scene = reinterpret_cast<physx::PxScene const&>(*scene_pod);
        physx::PxVec3 const& origin = reinterpret_cast<physx::PxVec3 const&>(*origin_pod);
        physx::PxVec3 const& unitDir = reinterpret_cast<physx::PxVec3 const&>(*unitDir_pod);
        auto outputFlags = physx::PxHitFlags(outputFlags_pod);
        physx::PxRaycastHit& hit = reinterpret_cast<physx::PxRaycastHit&>(*hit_pod);
        physx::PxQueryFilterData const& filterData = reinterpret_cast<physx::PxQueryFilterData const&>(*filterData_pod);
        physx::PxQueryFilterCallback* filterCall = reinterpret_cast<physx::PxQueryFilterCallback*>(filterCall_pod);
        physx::PxQueryCache const* cache = reinterpret_cast<physx::PxQueryCache const*>(cache_pod);
        bool return_val = PxSceneQueryExt::raycastSingle(scene, origin, unitDir, distance, outputFlags, hit, filterData, filterCall, cache);
        return return_val;
    }

    int32_t PxSceneQueryExt_raycastMultiple(physx_PxScene const* scene_pod, physx_PxVec3 const* origin_pod, physx_PxVec3 const* unitDir_pod, float distance, PxHitFlags outputFlags_pod, physx_PxRaycastHit* hitBuffer_pod, uint32_t hitBufferSize, bool* blockingHit_pod, physx_PxQueryFilterData const* filterData_pod, physx_PxQueryFilterCallback* filterCall_pod, physx_PxQueryCache const* cache_pod) {
        physx::PxScene const& scene = reinterpret_cast<physx::PxScene const&>(*scene_pod);
        physx::PxVec3 const& origin = reinterpret_cast<physx::PxVec3 const&>(*origin_pod);
        physx::PxVec3 const& unitDir = reinterpret_cast<physx::PxVec3 const&>(*unitDir_pod);
        auto outputFlags = physx::PxHitFlags(outputFlags_pod);
        physx::PxRaycastHit* hitBuffer = reinterpret_cast<physx::PxRaycastHit*>(hitBuffer_pod);
        bool& blockingHit = *blockingHit_pod;
        physx::PxQueryFilterData const& filterData = reinterpret_cast<physx::PxQueryFilterData const&>(*filterData_pod);
        physx::PxQueryFilterCallback* filterCall = reinterpret_cast<physx::PxQueryFilterCallback*>(filterCall_pod);
        physx::PxQueryCache const* cache = reinterpret_cast<physx::PxQueryCache const*>(cache_pod);
        int32_t return_val = PxSceneQueryExt::raycastMultiple(scene, origin, unitDir, distance, outputFlags, hitBuffer, hitBufferSize, blockingHit, filterData, filterCall, cache);
        return return_val;
    }

    bool PxSceneQueryExt_sweepAny(physx_PxScene const* scene_pod, physx_PxGeometry const* geometry_pod, physx_PxTransform const* pose_pod, physx_PxVec3 const* unitDir_pod, float distance, PxHitFlags queryFlags_pod, physx_PxQueryHit* hit_pod, physx_PxQueryFilterData const* filterData_pod, physx_PxQueryFilterCallback* filterCall_pod, physx_PxQueryCache const* cache_pod, float inflation) {
        physx::PxScene const& scene = reinterpret_cast<physx::PxScene const&>(*scene_pod);
        physx::PxGeometry const& geometry = reinterpret_cast<physx::PxGeometry const&>(*geometry_pod);
        physx::PxTransform const& pose = reinterpret_cast<physx::PxTransform const&>(*pose_pod);
        physx::PxVec3 const& unitDir = reinterpret_cast<physx::PxVec3 const&>(*unitDir_pod);
        auto queryFlags = physx::PxHitFlags(queryFlags_pod);
        physx::PxQueryHit& hit = reinterpret_cast<physx::PxQueryHit&>(*hit_pod);
        physx::PxQueryFilterData const& filterData = reinterpret_cast<physx::PxQueryFilterData const&>(*filterData_pod);
        physx::PxQueryFilterCallback* filterCall = reinterpret_cast<physx::PxQueryFilterCallback*>(filterCall_pod);
        physx::PxQueryCache const* cache = reinterpret_cast<physx::PxQueryCache const*>(cache_pod);
        bool return_val = PxSceneQueryExt::sweepAny(scene, geometry, pose, unitDir, distance, queryFlags, hit, filterData, filterCall, cache, inflation);
        return return_val;
    }

    bool PxSceneQueryExt_sweepSingle(physx_PxScene const* scene_pod, physx_PxGeometry const* geometry_pod, physx_PxTransform const* pose_pod, physx_PxVec3 const* unitDir_pod, float distance, PxHitFlags outputFlags_pod, physx_PxSweepHit* hit_pod, physx_PxQueryFilterData const* filterData_pod, physx_PxQueryFilterCallback* filterCall_pod, physx_PxQueryCache const* cache_pod, float inflation) {
        physx::PxScene const& scene = reinterpret_cast<physx::PxScene const&>(*scene_pod);
        physx::PxGeometry const& geometry = reinterpret_cast<physx::PxGeometry const&>(*geometry_pod);
        physx::PxTransform const& pose = reinterpret_cast<physx::PxTransform const&>(*pose_pod);
        physx::PxVec3 const& unitDir = reinterpret_cast<physx::PxVec3 const&>(*unitDir_pod);
        auto outputFlags = physx::PxHitFlags(outputFlags_pod);
        physx::PxSweepHit& hit = reinterpret_cast<physx::PxSweepHit&>(*hit_pod);
        physx::PxQueryFilterData const& filterData = reinterpret_cast<physx::PxQueryFilterData const&>(*filterData_pod);
        physx::PxQueryFilterCallback* filterCall = reinterpret_cast<physx::PxQueryFilterCallback*>(filterCall_pod);
        physx::PxQueryCache const* cache = reinterpret_cast<physx::PxQueryCache const*>(cache_pod);
        bool return_val = PxSceneQueryExt::sweepSingle(scene, geometry, pose, unitDir, distance, outputFlags, hit, filterData, filterCall, cache, inflation);
        return return_val;
    }

    int32_t PxSceneQueryExt_sweepMultiple(physx_PxScene const* scene_pod, physx_PxGeometry const* geometry_pod, physx_PxTransform const* pose_pod, physx_PxVec3 const* unitDir_pod, float distance, PxHitFlags outputFlags_pod, physx_PxSweepHit* hitBuffer_pod, uint32_t hitBufferSize, bool* blockingHit_pod, physx_PxQueryFilterData const* filterData_pod, physx_PxQueryFilterCallback* filterCall_pod, physx_PxQueryCache const* cache_pod, float inflation) {
        physx::PxScene const& scene = reinterpret_cast<physx::PxScene const&>(*scene_pod);
        physx::PxGeometry const& geometry = reinterpret_cast<physx::PxGeometry const&>(*geometry_pod);
        physx::PxTransform const& pose = reinterpret_cast<physx::PxTransform const&>(*pose_pod);
        physx::PxVec3 const& unitDir = reinterpret_cast<physx::PxVec3 const&>(*unitDir_pod);
        auto outputFlags = physx::PxHitFlags(outputFlags_pod);
        physx::PxSweepHit* hitBuffer = reinterpret_cast<physx::PxSweepHit*>(hitBuffer_pod);
        bool& blockingHit = *blockingHit_pod;
        physx::PxQueryFilterData const& filterData = reinterpret_cast<physx::PxQueryFilterData const&>(*filterData_pod);
        physx::PxQueryFilterCallback* filterCall = reinterpret_cast<physx::PxQueryFilterCallback*>(filterCall_pod);
        physx::PxQueryCache const* cache = reinterpret_cast<physx::PxQueryCache const*>(cache_pod);
        int32_t return_val = PxSceneQueryExt::sweepMultiple(scene, geometry, pose, unitDir, distance, outputFlags, hitBuffer, hitBufferSize, blockingHit, filterData, filterCall, cache, inflation);
        return return_val;
    }

    int32_t PxSceneQueryExt_overlapMultiple(physx_PxScene const* scene_pod, physx_PxGeometry const* geometry_pod, physx_PxTransform const* pose_pod, physx_PxOverlapHit* hitBuffer_pod, uint32_t hitBufferSize, physx_PxQueryFilterData const* filterData_pod, physx_PxQueryFilterCallback* filterCall_pod) {
        physx::PxScene const& scene = reinterpret_cast<physx::PxScene const&>(*scene_pod);
        physx::PxGeometry const& geometry = reinterpret_cast<physx::PxGeometry const&>(*geometry_pod);
        physx::PxTransform const& pose = reinterpret_cast<physx::PxTransform const&>(*pose_pod);
        physx::PxOverlapHit* hitBuffer = reinterpret_cast<physx::PxOverlapHit*>(hitBuffer_pod);
        physx::PxQueryFilterData const& filterData = reinterpret_cast<physx::PxQueryFilterData const&>(*filterData_pod);
        physx::PxQueryFilterCallback* filterCall = reinterpret_cast<physx::PxQueryFilterCallback*>(filterCall_pod);
        int32_t return_val = PxSceneQueryExt::overlapMultiple(scene, geometry, pose, hitBuffer, hitBufferSize, filterData, filterCall);
        return return_val;
    }

    bool PxSceneQueryExt_overlapAny(physx_PxScene const* scene_pod, physx_PxGeometry const* geometry_pod, physx_PxTransform const* pose_pod, physx_PxOverlapHit* hit_pod, physx_PxQueryFilterData const* filterData_pod, physx_PxQueryFilterCallback* filterCall_pod) {
        physx::PxScene const& scene = reinterpret_cast<physx::PxScene const&>(*scene_pod);
        physx::PxGeometry const& geometry = reinterpret_cast<physx::PxGeometry const&>(*geometry_pod);
        physx::PxTransform const& pose = reinterpret_cast<physx::PxTransform const&>(*pose_pod);
        physx::PxOverlapHit& hit = reinterpret_cast<physx::PxOverlapHit&>(*hit_pod);
        physx::PxQueryFilterData const& filterData = reinterpret_cast<physx::PxQueryFilterData const&>(*filterData_pod);
        physx::PxQueryFilterCallback* filterCall = reinterpret_cast<physx::PxQueryFilterCallback*>(filterCall_pod);
        bool return_val = PxSceneQueryExt::overlapAny(scene, geometry, pose, hit, filterData, filterCall);
        return return_val;
    }

    void PxBatchQueryExt_release(physx_PxBatchQueryExt* self__pod) {
        physx::PxBatchQueryExt* self_ = reinterpret_cast<physx::PxBatchQueryExt*>(self__pod);
        self_->release();
    }

    physx_PxRaycastBuffer* PxBatchQueryExt_raycast(physx_PxBatchQueryExt* self__pod, physx_PxVec3 const* origin_pod, physx_PxVec3 const* unitDir_pod, float distance, uint16_t maxNbTouches, PxHitFlags hitFlags_pod, physx_PxQueryFilterData const* filterData_pod, physx_PxQueryCache const* cache_pod) {
        physx::PxBatchQueryExt* self_ = reinterpret_cast<physx::PxBatchQueryExt*>(self__pod);
        physx::PxVec3 const& origin = reinterpret_cast<physx::PxVec3 const&>(*origin_pod);
        physx::PxVec3 const& unitDir = reinterpret_cast<physx::PxVec3 const&>(*unitDir_pod);
        auto hitFlags = physx::PxHitFlags(hitFlags_pod);
        physx::PxQueryFilterData const& filterData = reinterpret_cast<physx::PxQueryFilterData const&>(*filterData_pod);
        physx::PxQueryCache const* cache = reinterpret_cast<physx::PxQueryCache const*>(cache_pod);
        physx::PxRaycastBuffer* return_val = self_->raycast(origin, unitDir, distance, maxNbTouches, hitFlags, filterData, cache);
        auto return_val_pod = reinterpret_cast<physx_PxRaycastBuffer*>(return_val);
        return return_val_pod;
    }

    physx_PxSweepBuffer* PxBatchQueryExt_sweep(physx_PxBatchQueryExt* self__pod, physx_PxGeometry const* geometry_pod, physx_PxTransform const* pose_pod, physx_PxVec3 const* unitDir_pod, float distance, uint16_t maxNbTouches, PxHitFlags hitFlags_pod, physx_PxQueryFilterData const* filterData_pod, physx_PxQueryCache const* cache_pod, float inflation) {
        physx::PxBatchQueryExt* self_ = reinterpret_cast<physx::PxBatchQueryExt*>(self__pod);
        physx::PxGeometry const& geometry = reinterpret_cast<physx::PxGeometry const&>(*geometry_pod);
        physx::PxTransform const& pose = reinterpret_cast<physx::PxTransform const&>(*pose_pod);
        physx::PxVec3 const& unitDir = reinterpret_cast<physx::PxVec3 const&>(*unitDir_pod);
        auto hitFlags = physx::PxHitFlags(hitFlags_pod);
        physx::PxQueryFilterData const& filterData = reinterpret_cast<physx::PxQueryFilterData const&>(*filterData_pod);
        physx::PxQueryCache const* cache = reinterpret_cast<physx::PxQueryCache const*>(cache_pod);
        physx::PxSweepBuffer* return_val = self_->sweep(geometry, pose, unitDir, distance, maxNbTouches, hitFlags, filterData, cache, inflation);
        auto return_val_pod = reinterpret_cast<physx_PxSweepBuffer*>(return_val);
        return return_val_pod;
    }

    physx_PxOverlapBuffer* PxBatchQueryExt_overlap(physx_PxBatchQueryExt* self__pod, physx_PxGeometry const* geometry_pod, physx_PxTransform const* pose_pod, uint16_t maxNbTouches, physx_PxQueryFilterData const* filterData_pod, physx_PxQueryCache const* cache_pod) {
        physx::PxBatchQueryExt* self_ = reinterpret_cast<physx::PxBatchQueryExt*>(self__pod);
        physx::PxGeometry const& geometry = reinterpret_cast<physx::PxGeometry const&>(*geometry_pod);
        physx::PxTransform const& pose = reinterpret_cast<physx::PxTransform const&>(*pose_pod);
        physx::PxQueryFilterData const& filterData = reinterpret_cast<physx::PxQueryFilterData const&>(*filterData_pod);
        physx::PxQueryCache const* cache = reinterpret_cast<physx::PxQueryCache const*>(cache_pod);
        physx::PxOverlapBuffer* return_val = self_->overlap(geometry, pose, maxNbTouches, filterData, cache);
        auto return_val_pod = reinterpret_cast<physx_PxOverlapBuffer*>(return_val);
        return return_val_pod;
    }

    void PxBatchQueryExt_execute(physx_PxBatchQueryExt* self__pod) {
        physx::PxBatchQueryExt* self_ = reinterpret_cast<physx::PxBatchQueryExt*>(self__pod);
        self_->execute();
    }

    physx_PxBatchQueryExt* phys_PxCreateBatchQueryExt(physx_PxScene const* scene_pod, physx_PxQueryFilterCallback* queryFilterCallback_pod, uint32_t maxNbRaycasts, uint32_t maxNbRaycastTouches, uint32_t maxNbSweeps, uint32_t maxNbSweepTouches, uint32_t maxNbOverlaps, uint32_t maxNbOverlapTouches) {
        physx::PxScene const& scene = reinterpret_cast<physx::PxScene const&>(*scene_pod);
        physx::PxQueryFilterCallback* queryFilterCallback = reinterpret_cast<physx::PxQueryFilterCallback*>(queryFilterCallback_pod);
        physx::PxBatchQueryExt* return_val = PxCreateBatchQueryExt(scene, queryFilterCallback, maxNbRaycasts, maxNbRaycastTouches, maxNbSweeps, maxNbSweepTouches, maxNbOverlaps, maxNbOverlapTouches);
        auto return_val_pod = reinterpret_cast<physx_PxBatchQueryExt*>(return_val);
        return return_val_pod;
    }

    physx_PxBatchQueryExt* phys_PxCreateBatchQueryExt_1(physx_PxScene const* scene_pod, physx_PxQueryFilterCallback* queryFilterCallback_pod, physx_PxRaycastBuffer* raycastBuffers_pod, uint32_t maxNbRaycasts, physx_PxRaycastHit* raycastTouches_pod, uint32_t maxNbRaycastTouches, physx_PxSweepBuffer* sweepBuffers_pod, uint32_t maxNbSweeps, physx_PxSweepHit* sweepTouches_pod, uint32_t maxNbSweepTouches, physx_PxOverlapBuffer* overlapBuffers_pod, uint32_t maxNbOverlaps, physx_PxOverlapHit* overlapTouches_pod, uint32_t maxNbOverlapTouches) {
        physx::PxScene const& scene = reinterpret_cast<physx::PxScene const&>(*scene_pod);
        physx::PxQueryFilterCallback* queryFilterCallback = reinterpret_cast<physx::PxQueryFilterCallback*>(queryFilterCallback_pod);
        physx::PxRaycastBuffer* raycastBuffers = reinterpret_cast<physx::PxRaycastBuffer*>(raycastBuffers_pod);
        physx::PxRaycastHit* raycastTouches = reinterpret_cast<physx::PxRaycastHit*>(raycastTouches_pod);
        physx::PxSweepBuffer* sweepBuffers = reinterpret_cast<physx::PxSweepBuffer*>(sweepBuffers_pod);
        physx::PxSweepHit* sweepTouches = reinterpret_cast<physx::PxSweepHit*>(sweepTouches_pod);
        physx::PxOverlapBuffer* overlapBuffers = reinterpret_cast<physx::PxOverlapBuffer*>(overlapBuffers_pod);
        physx::PxOverlapHit* overlapTouches = reinterpret_cast<physx::PxOverlapHit*>(overlapTouches_pod);
        physx::PxBatchQueryExt* return_val = PxCreateBatchQueryExt(scene, queryFilterCallback, raycastBuffers, maxNbRaycasts, raycastTouches, maxNbRaycastTouches, sweepBuffers, maxNbSweeps, sweepTouches, maxNbSweepTouches, overlapBuffers, maxNbOverlaps, overlapTouches, maxNbOverlapTouches);
        auto return_val_pod = reinterpret_cast<physx_PxBatchQueryExt*>(return_val);
        return return_val_pod;
    }

    physx_PxSceneQuerySystem* phys_PxCreateExternalSceneQuerySystem(physx_PxSceneQueryDesc const* desc_pod, uint64_t contextID) {
        physx::PxSceneQueryDesc const& desc = reinterpret_cast<physx::PxSceneQueryDesc const&>(*desc_pod);
        physx::PxSceneQuerySystem* return_val = PxCreateExternalSceneQuerySystem(desc, contextID);
        auto return_val_pod = reinterpret_cast<physx_PxSceneQuerySystem*>(return_val);
        return return_val_pod;
    }

    uint32_t PxCustomSceneQuerySystem_addPruner(physx_PxCustomSceneQuerySystem* self__pod, PxPruningStructureType primaryType_pod, PxDynamicTreeSecondaryPruner secondaryType_pod, uint32_t preallocated) {
        physx::PxCustomSceneQuerySystem* self_ = reinterpret_cast<physx::PxCustomSceneQuerySystem*>(self__pod);
        auto primaryType = static_cast<physx::PxPruningStructureType::Enum>(primaryType_pod);
        auto secondaryType = static_cast<physx::PxDynamicTreeSecondaryPruner::Enum>(secondaryType_pod);
        uint32_t return_val = self_->addPruner(primaryType, secondaryType, preallocated);
        return return_val;
    }

    uint32_t PxCustomSceneQuerySystem_startCustomBuildstep(physx_PxCustomSceneQuerySystem* self__pod) {
        physx::PxCustomSceneQuerySystem* self_ = reinterpret_cast<physx::PxCustomSceneQuerySystem*>(self__pod);
        uint32_t return_val = self_->startCustomBuildstep();
        return return_val;
    }

    void PxCustomSceneQuerySystem_customBuildstep(physx_PxCustomSceneQuerySystem* self__pod, uint32_t index) {
        physx::PxCustomSceneQuerySystem* self_ = reinterpret_cast<physx::PxCustomSceneQuerySystem*>(self__pod);
        self_->customBuildstep(index);
    }

    void PxCustomSceneQuerySystem_finishCustomBuildstep(physx_PxCustomSceneQuerySystem* self__pod) {
        physx::PxCustomSceneQuerySystem* self_ = reinterpret_cast<physx::PxCustomSceneQuerySystem*>(self__pod);
        self_->finishCustomBuildstep();
    }

    void PxCustomSceneQuerySystemAdapter_delete(physx_PxCustomSceneQuerySystemAdapter* self__pod) {
        physx::PxCustomSceneQuerySystemAdapter* self_ = reinterpret_cast<physx::PxCustomSceneQuerySystemAdapter*>(self__pod);
        delete self_;
    }

    uint32_t PxCustomSceneQuerySystemAdapter_getPrunerIndex(physx_PxCustomSceneQuerySystemAdapter const* self__pod, physx_PxRigidActor const* actor_pod, physx_PxShape const* shape_pod) {
        physx::PxCustomSceneQuerySystemAdapter const* self_ = reinterpret_cast<physx::PxCustomSceneQuerySystemAdapter const*>(self__pod);
        physx::PxRigidActor const& actor = reinterpret_cast<physx::PxRigidActor const&>(*actor_pod);
        physx::PxShape const& shape = reinterpret_cast<physx::PxShape const&>(*shape_pod);
        uint32_t return_val = self_->getPrunerIndex(actor, shape);
        return return_val;
    }

    bool PxCustomSceneQuerySystemAdapter_processPruner(physx_PxCustomSceneQuerySystemAdapter const* self__pod, uint32_t prunerIndex, physx_PxQueryThreadContext const* context_pod, physx_PxQueryFilterData const* filterData_pod, physx_PxQueryFilterCallback* filterCall_pod) {
        physx::PxCustomSceneQuerySystemAdapter const* self_ = reinterpret_cast<physx::PxCustomSceneQuerySystemAdapter const*>(self__pod);
        physx::PxQueryThreadContext const* context = reinterpret_cast<physx::PxQueryThreadContext const*>(context_pod);
        physx::PxQueryFilterData const& filterData = reinterpret_cast<physx::PxQueryFilterData const&>(*filterData_pod);
        physx::PxQueryFilterCallback* filterCall = reinterpret_cast<physx::PxQueryFilterCallback*>(filterCall_pod);
        bool return_val = self_->processPruner(prunerIndex, context, filterData, filterCall);
        return return_val;
    }

    physx_PxCustomSceneQuerySystem* phys_PxCreateCustomSceneQuerySystem(PxSceneQueryUpdateMode sceneQueryUpdateMode_pod, uint64_t contextID, physx_PxCustomSceneQuerySystemAdapter const* adapter_pod, bool usesTreeOfPruners) {
        auto sceneQueryUpdateMode = static_cast<physx::PxSceneQueryUpdateMode::Enum>(sceneQueryUpdateMode_pod);
        physx::PxCustomSceneQuerySystemAdapter const& adapter = reinterpret_cast<physx::PxCustomSceneQuerySystemAdapter const&>(*adapter_pod);
        physx::PxCustomSceneQuerySystem* return_val = PxCreateCustomSceneQuerySystem(sceneQueryUpdateMode, contextID, adapter, usesTreeOfPruners);
        auto return_val_pod = reinterpret_cast<physx_PxCustomSceneQuerySystem*>(return_val);
        return return_val_pod;
    }

    uint32_t phys_PxFindFaceIndex(physx_PxConvexMeshGeometry const* convexGeom_pod, physx_PxTransform const* geomPose_pod, physx_PxVec3 const* impactPos_pod, physx_PxVec3 const* unitDir_pod) {
        physx::PxConvexMeshGeometry const& convexGeom = reinterpret_cast<physx::PxConvexMeshGeometry const&>(*convexGeom_pod);
        physx::PxTransform const& geomPose = reinterpret_cast<physx::PxTransform const&>(*geomPose_pod);
        physx::PxVec3 const& impactPos = reinterpret_cast<physx::PxVec3 const&>(*impactPos_pod);
        physx::PxVec3 const& unitDir = reinterpret_cast<physx::PxVec3 const&>(*unitDir_pod);
        uint32_t return_val = PxFindFaceIndex(convexGeom, geomPose, impactPos, unitDir);
        return return_val;
    }

    bool PxPoissonSampler_setSamplingRadius(physx_PxPoissonSampler* self__pod, float samplingRadius) {
        physx::PxPoissonSampler* self_ = reinterpret_cast<physx::PxPoissonSampler*>(self__pod);
        bool return_val = self_->setSamplingRadius(samplingRadius);
        return return_val;
    }

    void PxPoissonSampler_addSamplesInSphere(physx_PxPoissonSampler* self__pod, physx_PxVec3 const* sphereCenter_pod, float sphereRadius, bool createVolumeSamples) {
        physx::PxPoissonSampler* self_ = reinterpret_cast<physx::PxPoissonSampler*>(self__pod);
        physx::PxVec3 const& sphereCenter = reinterpret_cast<physx::PxVec3 const&>(*sphereCenter_pod);
        self_->addSamplesInSphere(sphereCenter, sphereRadius, createVolumeSamples);
    }

    void PxPoissonSampler_addSamplesInBox(physx_PxPoissonSampler* self__pod, physx_PxBounds3 const* axisAlignedBox_pod, physx_PxQuat const* boxOrientation_pod, bool createVolumeSamples) {
        physx::PxPoissonSampler* self_ = reinterpret_cast<physx::PxPoissonSampler*>(self__pod);
        physx::PxBounds3 const& axisAlignedBox = reinterpret_cast<physx::PxBounds3 const&>(*axisAlignedBox_pod);
        physx::PxQuat const& boxOrientation = reinterpret_cast<physx::PxQuat const&>(*boxOrientation_pod);
        self_->addSamplesInBox(axisAlignedBox, boxOrientation, createVolumeSamples);
    }

    void PxPoissonSampler_delete(physx_PxPoissonSampler* self__pod) {
        physx::PxPoissonSampler* self_ = reinterpret_cast<physx::PxPoissonSampler*>(self__pod);
        delete self_;
    }

    physx_PxPoissonSampler* phys_PxCreateShapeSampler(physx_PxGeometry const* geometry_pod, physx_PxTransform const* transform_pod, physx_PxBounds3 const* worldBounds_pod, float initialSamplingRadius, int32_t numSampleAttemptsAroundPoint) {
        physx::PxGeometry const& geometry = reinterpret_cast<physx::PxGeometry const&>(*geometry_pod);
        physx::PxTransform const& transform = reinterpret_cast<physx::PxTransform const&>(*transform_pod);
        physx::PxBounds3 const& worldBounds = reinterpret_cast<physx::PxBounds3 const&>(*worldBounds_pod);
        physx::PxPoissonSampler* return_val = PxCreateShapeSampler(geometry, transform, worldBounds, initialSamplingRadius, numSampleAttemptsAroundPoint);
        auto return_val_pod = reinterpret_cast<physx_PxPoissonSampler*>(return_val);
        return return_val_pod;
    }

    bool PxTriangleMeshPoissonSampler_isPointInTriangleMesh(physx_PxTriangleMeshPoissonSampler* self__pod, physx_PxVec3 const* p_pod) {
        physx::PxTriangleMeshPoissonSampler* self_ = reinterpret_cast<physx::PxTriangleMeshPoissonSampler*>(self__pod);
        physx::PxVec3 const& p = reinterpret_cast<physx::PxVec3 const&>(*p_pod);
        bool return_val = self_->isPointInTriangleMesh(p);
        return return_val;
    }

    void PxTriangleMeshPoissonSampler_delete(physx_PxTriangleMeshPoissonSampler* self__pod) {
        physx::PxTriangleMeshPoissonSampler* self_ = reinterpret_cast<physx::PxTriangleMeshPoissonSampler*>(self__pod);
        delete self_;
    }

    physx_PxTriangleMeshPoissonSampler* phys_PxCreateTriangleMeshSampler(uint32_t const* triangles, uint32_t numTriangles, physx_PxVec3 const* vertices_pod, uint32_t numVertices, float initialSamplingRadius, int32_t numSampleAttemptsAroundPoint) {
        physx::PxVec3 const* vertices = reinterpret_cast<physx::PxVec3 const*>(vertices_pod);
        physx::PxTriangleMeshPoissonSampler* return_val = PxCreateTriangleMeshSampler(triangles, numTriangles, vertices, numVertices, initialSamplingRadius, numSampleAttemptsAroundPoint);
        auto return_val_pod = reinterpret_cast<physx_PxTriangleMeshPoissonSampler*>(return_val);
        return return_val_pod;
    }

    int32_t PxTetrahedronMeshExt_findTetrahedronContainingPoint(physx_PxTetrahedronMesh const* mesh_pod, physx_PxVec3 const* point_pod, physx_PxVec4* bary_pod, float tolerance) {
        physx::PxTetrahedronMesh const* mesh = reinterpret_cast<physx::PxTetrahedronMesh const*>(mesh_pod);
        physx::PxVec3 const& point = reinterpret_cast<physx::PxVec3 const&>(*point_pod);
        physx::PxVec4& bary = reinterpret_cast<physx::PxVec4&>(*bary_pod);
        int32_t return_val = PxTetrahedronMeshExt::findTetrahedronContainingPoint(mesh, point, bary, tolerance);
        return return_val;
    }

    int32_t PxTetrahedronMeshExt_findTetrahedronClosestToPoint(physx_PxTetrahedronMesh const* mesh_pod, physx_PxVec3 const* point_pod, physx_PxVec4* bary_pod) {
        physx::PxTetrahedronMesh const* mesh = reinterpret_cast<physx::PxTetrahedronMesh const*>(mesh_pod);
        physx::PxVec3 const& point = reinterpret_cast<physx::PxVec3 const&>(*point_pod);
        physx::PxVec4& bary = reinterpret_cast<physx::PxVec4&>(*bary_pod);
        int32_t return_val = PxTetrahedronMeshExt::findTetrahedronClosestToPoint(mesh, point, bary);
        return return_val;
    }

    bool phys_PxInitExtensions(physx_PxPhysics* physics_pod, physx_PxPvd* pvd_pod) {
        physx::PxPhysics& physics = reinterpret_cast<physx::PxPhysics&>(*physics_pod);
        physx::PxPvd* pvd = reinterpret_cast<physx::PxPvd*>(pvd_pod);
        bool return_val = PxInitExtensions(physics, pvd);
        return return_val;
    }

    void phys_PxCloseExtensions() {
        PxCloseExtensions();
    }

    physx_PxRepXObject PxRepXObject_new(char const* inTypeName, void const* inSerializable, uint64_t inId) {
        PxRepXObject return_val(inTypeName, inSerializable, inId);
        physx_PxRepXObject return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxRepXObject_isValid(physx_PxRepXObject const* self__pod) {
        physx::PxRepXObject const* self_ = reinterpret_cast<physx::PxRepXObject const*>(self__pod);
        bool return_val = self_->isValid();
        return return_val;
    }

    physx_PxRepXInstantiationArgs PxRepXInstantiationArgs_new(physx_PxPhysics* inPhysics_pod, physx_PxCooking* inCooking_pod, physx_PxStringTable* inStringTable_pod) {
        physx::PxPhysics& inPhysics = reinterpret_cast<physx::PxPhysics&>(*inPhysics_pod);
        physx::PxCooking* inCooking = reinterpret_cast<physx::PxCooking*>(inCooking_pod);
        physx::PxStringTable* inStringTable = reinterpret_cast<physx::PxStringTable*>(inStringTable_pod);
        PxRepXInstantiationArgs return_val(inPhysics, inCooking, inStringTable);
        physx_PxRepXInstantiationArgs return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    char const* PxRepXSerializer_getTypeName(physx_PxRepXSerializer* self__pod) {
        physx::PxRepXSerializer* self_ = reinterpret_cast<physx::PxRepXSerializer*>(self__pod);
        char const* return_val = self_->getTypeName();
        return return_val;
    }

    void PxRepXSerializer_objectToFile(physx_PxRepXSerializer* self__pod, physx_PxRepXObject const* inLiveObject_pod, physx_PxCollection* inCollection_pod, physx_XmlWriter* inWriter_pod, physx_MemoryBuffer* inTempBuffer_pod, physx_PxRepXInstantiationArgs* inArgs_pod) {
        physx::PxRepXSerializer* self_ = reinterpret_cast<physx::PxRepXSerializer*>(self__pod);
        physx::PxRepXObject const& inLiveObject = reinterpret_cast<physx::PxRepXObject const&>(*inLiveObject_pod);
        physx::PxCollection* inCollection = reinterpret_cast<physx::PxCollection*>(inCollection_pod);
        physx::XmlWriter& inWriter = reinterpret_cast<physx::XmlWriter&>(*inWriter_pod);
        physx::MemoryBuffer& inTempBuffer = reinterpret_cast<physx::MemoryBuffer&>(*inTempBuffer_pod);
        physx::PxRepXInstantiationArgs& inArgs = reinterpret_cast<physx::PxRepXInstantiationArgs&>(*inArgs_pod);
        self_->objectToFile(inLiveObject, inCollection, inWriter, inTempBuffer, inArgs);
    }

    physx_PxRepXObject PxRepXSerializer_fileToObject(physx_PxRepXSerializer* self__pod, physx_XmlReader* inReader_pod, physx_XmlMemoryAllocator* inAllocator_pod, physx_PxRepXInstantiationArgs* inArgs_pod, physx_PxCollection* inCollection_pod) {
        physx::PxRepXSerializer* self_ = reinterpret_cast<physx::PxRepXSerializer*>(self__pod);
        physx::XmlReader& inReader = reinterpret_cast<physx::XmlReader&>(*inReader_pod);
        physx::XmlMemoryAllocator& inAllocator = reinterpret_cast<physx::XmlMemoryAllocator&>(*inAllocator_pod);
        physx::PxRepXInstantiationArgs& inArgs = reinterpret_cast<physx::PxRepXInstantiationArgs&>(*inArgs_pod);
        physx::PxCollection* inCollection = reinterpret_cast<physx::PxCollection*>(inCollection_pod);
        physx::PxRepXObject return_val = self_->fileToObject(inReader, inAllocator, inArgs, inCollection);
        physx_PxRepXObject return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    bool PxPvd_connect(physx_PxPvd* self__pod, physx_PxPvdTransport* transport_pod, PxPvdInstrumentationFlags flags_pod) {
        physx::PxPvd* self_ = reinterpret_cast<physx::PxPvd*>(self__pod);
        physx::PxPvdTransport& transport = reinterpret_cast<physx::PxPvdTransport&>(*transport_pod);
        auto flags = physx::PxPvdInstrumentationFlags(flags_pod);
        bool return_val = self_->connect(transport, flags);
        return return_val;
    }

    void PxPvd_disconnect(physx_PxPvd* self__pod) {
        physx::PxPvd* self_ = reinterpret_cast<physx::PxPvd*>(self__pod);
        self_->disconnect();
    }

    bool PxPvd_isConnected(physx_PxPvd* self__pod, bool useCachedStatus) {
        physx::PxPvd* self_ = reinterpret_cast<physx::PxPvd*>(self__pod);
        bool return_val = self_->isConnected(useCachedStatus);
        return return_val;
    }

    physx_PxPvdTransport* PxPvd_getTransport(physx_PxPvd* self__pod) {
        physx::PxPvd* self_ = reinterpret_cast<physx::PxPvd*>(self__pod);
        physx::PxPvdTransport* return_val = self_->getTransport();
        auto return_val_pod = reinterpret_cast<physx_PxPvdTransport*>(return_val);
        return return_val_pod;
    }

    PxPvdInstrumentationFlags PxPvd_getInstrumentationFlags(physx_PxPvd* self__pod) {
        physx::PxPvd* self_ = reinterpret_cast<physx::PxPvd*>(self__pod);
        physx::PxPvdInstrumentationFlags return_val = self_->getInstrumentationFlags();
        PxPvdInstrumentationFlags return_val_pod;
        memcpy(&return_val_pod, &return_val, sizeof(return_val_pod));
        return return_val_pod;
    }

    void PxPvd_release(physx_PxPvd* self__pod) {
        physx::PxPvd* self_ = reinterpret_cast<physx::PxPvd*>(self__pod);
        self_->release();
    }

    physx_PxPvd* phys_PxCreatePvd(physx_PxFoundation* foundation_pod) {
        physx::PxFoundation& foundation = reinterpret_cast<physx::PxFoundation&>(*foundation_pod);
        physx::PxPvd* return_val = PxCreatePvd(foundation);
        auto return_val_pod = reinterpret_cast<physx_PxPvd*>(return_val);
        return return_val_pod;
    }

    bool PxPvdTransport_connect(physx_PxPvdTransport* self__pod) {
        physx::PxPvdTransport* self_ = reinterpret_cast<physx::PxPvdTransport*>(self__pod);
        bool return_val = self_->connect();
        return return_val;
    }

    void PxPvdTransport_disconnect(physx_PxPvdTransport* self__pod) {
        physx::PxPvdTransport* self_ = reinterpret_cast<physx::PxPvdTransport*>(self__pod);
        self_->disconnect();
    }

    bool PxPvdTransport_isConnected(physx_PxPvdTransport* self__pod) {
        physx::PxPvdTransport* self_ = reinterpret_cast<physx::PxPvdTransport*>(self__pod);
        bool return_val = self_->isConnected();
        return return_val;
    }

    bool PxPvdTransport_write(physx_PxPvdTransport* self__pod, uint8_t const* inBytes, uint32_t inLength) {
        physx::PxPvdTransport* self_ = reinterpret_cast<physx::PxPvdTransport*>(self__pod);
        bool return_val = self_->write(inBytes, inLength);
        return return_val;
    }

    physx_PxPvdTransport* PxPvdTransport_lock(physx_PxPvdTransport* self__pod) {
        physx::PxPvdTransport* self_ = reinterpret_cast<physx::PxPvdTransport*>(self__pod);
        physx::PxPvdTransport& return_val = self_->lock();
        auto return_val_pod = reinterpret_cast<physx_PxPvdTransport*>(&return_val);
        return return_val_pod;
    }

    void PxPvdTransport_unlock(physx_PxPvdTransport* self__pod) {
        physx::PxPvdTransport* self_ = reinterpret_cast<physx::PxPvdTransport*>(self__pod);
        self_->unlock();
    }

    void PxPvdTransport_flush(physx_PxPvdTransport* self__pod) {
        physx::PxPvdTransport* self_ = reinterpret_cast<physx::PxPvdTransport*>(self__pod);
        self_->flush();
    }

    uint64_t PxPvdTransport_getWrittenDataSize(physx_PxPvdTransport* self__pod) {
        physx::PxPvdTransport* self_ = reinterpret_cast<physx::PxPvdTransport*>(self__pod);
        uint64_t return_val = self_->getWrittenDataSize();
        return return_val;
    }

    void PxPvdTransport_release(physx_PxPvdTransport* self__pod) {
        physx::PxPvdTransport* self_ = reinterpret_cast<physx::PxPvdTransport*>(self__pod);
        self_->release();
    }

    physx_PxPvdTransport* phys_PxDefaultPvdSocketTransportCreate(char const* host, int32_t port, uint32_t timeoutInMilliseconds) {
        physx::PxPvdTransport* return_val = PxDefaultPvdSocketTransportCreate(host, port, timeoutInMilliseconds);
        auto return_val_pod = reinterpret_cast<physx_PxPvdTransport*>(return_val);
        return return_val_pod;
    }

    physx_PxPvdTransport* phys_PxDefaultPvdFileTransportCreate(char const* name) {
        physx::PxPvdTransport* return_val = PxDefaultPvdFileTransportCreate(name);
        auto return_val_pod = reinterpret_cast<physx_PxPvdTransport*>(return_val);
        return return_val_pod;
    }

}
