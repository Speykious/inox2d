/*!
Inochi2D node types to Inox2D components:
- Node -> (Nothing)
- Part -> Drawable + TexturedMesh + Mesh
- Composite -> Drawable + Composite
- SimplePhysics -> SimplePhysics
- MeshGroup -> MeshGroup + Mesh
- Custom nodes by inheritance -> Custom nodes by composition
*/

use glam::{Mat4, Vec2, Vec3};

use crate::math::deform::Deform;
use crate::node::{InoxNodeUuid, TransformOffset};
use crate::params::ParamUuid;
use crate::physics::{
	pendulum::{rigid::RigidPendulum, spring::SpringPendulum},
	runge_kutta::PhysicsState,
};
use crate::texture::TextureId;

/* --- COMPOSITE --- */

/// If has this as a component, the node should composite all children
///
/// Empty as only a marker, zsorted children list constructed later on demand
pub struct Composite {}

/* --- DRAWABLE --- */

/// If has this as a component, the node should render something
pub struct Drawable {
	pub blending: Blending,
	/// If Some, the node should consider masking when rendering
	pub masks: Option<Masks>,
}

pub struct Blending {
	pub mode: BlendMode,
	pub tint: Vec3,
	pub screen_tint: Vec3,
	pub opacity: f32,
}

#[derive(Default, PartialEq, Clone, Copy)]
pub enum BlendMode {
	/// Normal blending mode.
	#[default]
	Normal,
	/// Multiply blending mode.
	Multiply,
	/// Color Dodge.
	ColorDodge,
	/// Linear Dodge.
	LinearDodge,
	/// Screen.
	Screen,
	/// Clip to Lower.
	/// Special blending mode that clips the drawable
	/// to a lower rendered area.
	ClipToLower,
	/// Slice from Lower.
	/// Special blending mode that slices the drawable
	/// via a lower rendered area.
	/// (Basically inverse ClipToLower.)
	SliceFromLower,
}

impl BlendMode {
	pub const VALUES: [BlendMode; 7] = [
		BlendMode::Normal,
		BlendMode::Multiply,
		BlendMode::ColorDodge,
		BlendMode::LinearDodge,
		BlendMode::Screen,
		BlendMode::ClipToLower,
		BlendMode::SliceFromLower,
	];
}

pub struct Masks {
	pub threshold: f32,
	pub masks: Vec<Mask>,
}

impl Masks {
	/// Checks whether has masks of mode `MaskMode::Mask`.
	pub fn has_masks(&self) -> bool {
		self.masks.iter().any(|mask| mask.mode == MaskMode::Mask)
	}

	/// Checks whether has masks of mode `MaskMode::Dodge`.
	pub fn has_dodge_masks(&self) -> bool {
		self.masks.iter().any(|mask| mask.mode == MaskMode::Dodge)
	}
}

pub struct Mask {
	pub source: InoxNodeUuid,
	pub mode: MaskMode,
}

#[derive(PartialEq)]
pub enum MaskMode {
	/// The part should be masked by the drawables specified.
	Mask,
	/// The path should be dodge-masked by the drawables specified.
	Dodge,
}

/* --- SIMPLE PHYSICS --- */

/// If has this as a component, the node is capable of doing Inochi2D SimplePhysics simulations
#[derive(Clone)]
pub struct SimplePhysics {
	pub param: ParamUuid,
	pub model_type: PhysicsModel,
	pub map_mode: PhysicsParamMapMode,
	pub props: PhysicsProps,
	/// Whether physics system listens to local transform only.
	pub local_only: bool,
}

#[derive(Clone)]
pub enum PhysicsModel {
	RigidPendulum,
	SpringPendulum,
}

#[derive(Clone)]
pub enum PhysicsParamMapMode {
	AngleLength,
	XY,
	YX,
}

#[derive(Clone)]
pub struct PhysicsProps {
	/// Gravity scale (1.0 = puppet gravity)
	pub gravity: f32,
	/// Pendulum/spring rest length (pixels)
	pub length: f32,
	/// Resonant frequency (Hz)
	pub frequency: f32,
	/// Angular damping ratio
	pub angle_damping: f32,
	/// Length damping ratio
	pub length_damping: f32,
	pub output_scale: Vec2,
}

impl Default for PhysicsProps {
	fn default() -> Self {
		Self {
			gravity: 1.,
			length: 1.,
			frequency: 1.,
			angle_damping: 0.5,
			length_damping: 0.5,
			output_scale: Vec2::ONE,
		}
	}
}

/// Physical states for simulating a rigid pendulum.
#[derive(Default)]
pub(crate) struct RigidPendulumCtx {
	pub bob: Vec2,
	pub state: PhysicsState<2, RigidPendulum>,
}

/// Physical states for simulating a spring pendulum.
#[derive(Default)]
pub(crate) struct SpringPendulumCtx {
	pub state: PhysicsState<4, SpringPendulum>,
}

/* --- TEXTURED MESH --- */

/// If has this as a component, the node should render a deformed texture
pub struct TexturedMesh {
	pub tex_albedo: TextureId,
	pub tex_emissive: TextureId,
	pub tex_bumpmap: TextureId,
}

/* --- MESH --- */

/// A deformable mesh, deforming either textures (TexturedMesh nodes), or children (MeshGroup nodes)
pub struct Mesh {
	/// Vertices in the mesh.
	pub vertices: Vec<Vec2>,
	/// Base UVs.
	pub uvs: Vec<Vec2>,
	/// Indices in the mesh.
	pub indices: Vec<u16>,
	/// Origin of the mesh.
	pub origin: Vec2,
}

/* --- MESHGROUP --- */

pub struct MeshGroup {
	pub dynamic: bool,
	pub translate_children: bool,
}

/* --- DEFORM STACK --- */

/// Source of a deform.
#[derive(Hash, PartialEq, Eq, Copy, Clone)]
pub(crate) enum DeformSource {
	Param(ParamUuid),
	MeshGroup(InoxNodeUuid),
}

/// Internal component solving for deforms of a node.
/// Storing deforms specified by multiple sources to apply on one node for one frame.
///
/// Despite the name (this is respecting the ref impl), this is not in any way a stack.
/// The order of deforms being applied, or more generally speaking, the way multiple deforms adds up to be a single one, needs to be defined according to the spec.
pub(crate) struct DeformStack {
	/// this is a component so cannot use generics for the length.
	pub(crate) deform_len: usize,
	/// map of (src, (enabled, Deform)).
	/// On reset, only set enabled to false instead of clearing the map, as deforms from same sources tend to come in every frame.
	pub(crate) stack: std::collections::HashMap<DeformSource, (bool, Deform)>,
}

/* --- TRANSFORM STORE --- */

/// Internal component storing:
/// - Relative transform being determined in between frames.
/// - Absolute transform prepared from all relative transforms just before rendering.
#[derive(Default, Clone)]
pub struct TransformStore {
	pub absolute: Mat4,
	pub relative: TransformOffset,
}

/* --- ZSORT --- */

/// Component holding zsort values that may be modified across frames.
// only one value instead of absolute + relative as in TransformStore, cause inheritance of zsort (+) is commutative
#[derive(Default)]
pub(crate) struct ZSort(pub f32);

// so ZSort automatically gets the `.total_cmp()` of `f32`
impl std::ops::Deref for ZSort {
	type Target = f32;

	fn deref(&self) -> &Self::Target {
		&self.0
	}
}
