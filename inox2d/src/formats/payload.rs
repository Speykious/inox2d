use std::collections::HashMap;

use glam::{vec2, vec3, Vec2};
use json::JsonValue;

use crate::math::interp::InterpolateMode;
use crate::math::matrix::{Matrix2d, Matrix2dFromSliceVecsError};
use crate::math::transform::TransformOffset;
use crate::node::components::*;
use crate::node::{InoxNode, InoxNodeUuid};
use crate::params::{AxisPoints, Binding, BindingValues, Param, ParamUuid};
use crate::physics::PuppetPhysics;
use crate::puppet::{meta::*, Puppet};
use crate::texture::TextureId;

use super::f32s_as_vec2s;
use super::json::{JsonError, JsonObject, SerialExtend};

pub type InoxParseResult<T> = Result<T, InoxParseError>;

#[derive(Debug, Clone, thiserror::Error)]
pub enum InoxParseError {
	#[error(transparent)]
	JsonError(#[from] JsonError),
	#[error("Unknown node type {0:?}")]
	UnknownNodeType(String),
	#[error("Unknown param name {0:?}")]
	UnknownParamName(String),
	#[error("No albedo texture")]
	NoAlbedoTexture,
	#[error(transparent)]
	InvalidMatrix2dData(#[from] Matrix2dFromSliceVecsError),
	#[error("Unknown param map mode {0:?}")]
	UnknownParamMapMode(String),
	#[error("Unknown mask mode {0:?}")]
	UnknownMaskMode(String),
	#[error("Unknown interpolate mode {0:?}")]
	UnknownInterpolateMode(String),
	#[error("Unknown allowed users {0:?}")]
	UnknownPuppetAllowedUsers(String),
	#[error("Unknown allowed redistribution {0:?}")]
	UnknownPuppetAllowedRedistribution(String),
	#[error("Unknown allowed modification {0:?}")]
	UnknownPuppetAllowedModification(String),
	#[error("Expected even number of floats in list, got {0}")]
	OddNumberOfFloatsInList(usize),
	#[error("Expected 2 floats in list, got {0}")]
	Not2FloatsInList(usize),
}

// json structure helpers

impl InoxParseError {
	pub fn nested(self, key: &str) -> Self {
		match self {
			InoxParseError::JsonError(err) => InoxParseError::JsonError(err.nested(key)),
			_ => self,
		}
	}
}

fn vals<T>(key: &str, res: InoxParseResult<T>) -> InoxParseResult<T> {
	res.map_err(|e| e.nested(key))
}

fn as_nested_list(index: usize, val: &json::JsonValue) -> InoxParseResult<&[json::JsonValue]> {
	match val {
		json::JsonValue::Array(arr) => Ok(arr),
		_ => Err(InoxParseError::JsonError(JsonError::ValueIsNotList(index.to_string()))),
	}
}

fn as_object<'file>(msg: &str, val: &'file JsonValue) -> InoxParseResult<JsonObject<'file>> {
	if let Some(obj) = val.as_object() {
		Ok(JsonObject(obj))
	} else {
		Err(InoxParseError::JsonError(JsonError::ValueIsNotObject(msg.to_owned())))
	}
}

// node deserialization

struct ParsedNode<'file> {
	node: InoxNode,
	ty: &'file str,
	data: JsonObject<'file>,
	children: &'file [JsonValue],
}

fn deserialize_node(obj: JsonObject) -> InoxParseResult<ParsedNode> {
	Ok(ParsedNode {
		node: InoxNode {
			uuid: InoxNodeUuid(obj.get_u32("uuid")?),
			name: obj.get_str("name")?.to_owned(),
			enabled: obj.get_bool("enabled")?,
			zsort: obj.get_f32("zsort")?,
			trans_offset: vals("transform", deserialize_transform(obj.get_object("transform")?))?,
			lock_to_root: obj.get_bool("lockToRoot")?,
		},
		ty: obj.get_str("type")?,
		data: obj,
		children: { obj.get_list("children").unwrap_or(&[]) },
	})
}

// components deserialization

fn deserialize_textured_mesh(obj: JsonObject) -> InoxParseResult<TexturedMesh> {
	let (tex_albedo, tex_emissive, tex_bumpmap) = {
		let textures = obj.get_list("textures")?;

		let tex_albedo = match textures.first().ok_or(InoxParseError::NoAlbedoTexture)?.as_number() {
			Some(val) => val
				.try_into()
				.map(TextureId)
				.map_err(|_| InoxParseError::JsonError(JsonError::ParseIntError("0".to_owned()).nested("textures")))?,
			None => return Err(InoxParseError::NoAlbedoTexture),
		};

		let tex_emissive = match textures.get(1).and_then(JsonValue::as_number) {
			Some(val) => (val.try_into())
				// Map u32::MAX to nothing
				.map(|val| if val == u32::MAX as usize { 0 } else { val })
				.map(TextureId)
				.map_err(|_| InoxParseError::JsonError(JsonError::ParseIntError("1".to_owned()).nested("textures")))?,
			None => TextureId(0),
		};

		let tex_bumpmap = match textures.get(2).and_then(JsonValue::as_number) {
			Some(val) => (val.try_into())
				// Map u32::MAX to nothing
				.map(|val| if val == u32::MAX as usize { 0 } else { val })
				.map(TextureId)
				.map_err(|_| InoxParseError::JsonError(JsonError::ParseIntError("2".to_owned()).nested("textures")))?,
			None => TextureId(0),
		};

		(tex_albedo, tex_emissive, tex_bumpmap)
	};

	Ok(TexturedMesh {
		tex_albedo,
		tex_emissive,
		tex_bumpmap,
	})
}

fn deserialize_simple_physics(obj: JsonObject) -> InoxParseResult<SimplePhysics> {
	Ok(SimplePhysics {
		param: ParamUuid(obj.get_u32("param")?),

		model_type: match obj.get_str("model_type")? {
			"Pendulum" => PhysicsModel::RigidPendulum,
			"SpringPendulum" => PhysicsModel::SpringPendulum,
			a => todo!("{}", a),
		},
		map_mode: match obj.get_str("map_mode")? {
			"AngleLength" => PhysicsParamMapMode::AngleLength,
			"XY" => PhysicsParamMapMode::XY,
			"YX" => PhysicsParamMapMode::YX,
			unknown => return Err(InoxParseError::UnknownParamMapMode(unknown.to_owned())),
		},

		props: PhysicsProps {
			gravity: obj.get_f32("gravity")?,
			length: obj.get_f32("length")?,
			frequency: obj.get_f32("frequency")?,
			angle_damping: obj.get_f32("angle_damping")?,
			length_damping: obj.get_f32("length_damping")?,
			output_scale: obj.get_vec2("output_scale")?,
		},

		local_only: obj.get_bool("local_only").unwrap_or_default(),
	})
}

fn deserialize_drawable(obj: JsonObject) -> InoxParseResult<Drawable> {
	Ok(Drawable {
		blending: Blending {
			mode: match obj.get_str("blend_mode")? {
				"Normal" => BlendMode::Normal,
				"Multiply" => BlendMode::Multiply,
				"ColorDodge" => BlendMode::ColorDodge,
				"LinearDodge" => BlendMode::LinearDodge,
				"Screen" => BlendMode::Screen,
				"ClipToLower" => BlendMode::ClipToLower,
				"SliceFromLower" => BlendMode::SliceFromLower,
				_ => BlendMode::default(),
			},
			tint: obj.get_vec3("tint").unwrap_or(vec3(1.0, 1.0, 1.0)),
			screen_tint: obj.get_vec3("screenTint").unwrap_or(vec3(0.0, 0.0, 0.0)),
			opacity: obj.get_f32("opacity").unwrap_or(1.0),
		},
		masks: {
			if let Ok(masks) = obj.get_list("masks") {
				Some(Masks {
					threshold: obj.get_f32("mask_threshold").unwrap_or(0.5),
					masks: {
						let mut collection = Vec::<Mask>::new();
						for mask_obj in masks {
							let mask = deserialize_mask(as_object("mask", mask_obj)?)?;
							collection.push(mask);
						}
						collection
					},
				})
			} else {
				None
			}
		},
	})
}

fn deserialize_mesh(obj: JsonObject) -> InoxParseResult<Mesh> {
	Ok(Mesh {
		vertices: deserialize_vec2s_flat(obj.get_list("verts")?)?,
		uvs: deserialize_vec2s_flat(obj.get_list("uvs")?)?,
		indices: obj
			.get_list("indices")?
			.iter()
			.map_while(JsonValue::as_u16)
			.collect::<Vec<_>>(),
		origin: obj.get_vec2("origin").unwrap_or_default(),
	})
}

fn deserialize_mask(obj: JsonObject) -> InoxParseResult<Mask> {
	Ok(Mask {
		source: InoxNodeUuid(obj.get_u32("source")?),
		mode: match obj.get_str("mode")? {
			"Mask" => MaskMode::Mask,
			"DodgeMask" => MaskMode::Dodge,
			unknown => return Err(InoxParseError::UnknownMaskMode(unknown.to_owned())),
		},
	})
}

fn deserialize_transform(obj: JsonObject) -> InoxParseResult<TransformOffset> {
	Ok(TransformOffset {
		translation: obj.get_vec3("trans")?,
		rotation: obj.get_vec3("rot")?,
		scale: obj.get_vec2("scale")?,
		pixel_snap: obj.get_bool("pixel_snap").unwrap_or_default(),
	})
}

fn deserialize_meshgroup(obj: JsonObject) -> InoxParseResult<MeshGroup> {
	Ok(MeshGroup {
		dynamic: obj.get_bool("dynamic_deformation")?,
		translate_children: obj.get_bool("translate_children")?,
	})
}

// deserialization helpers

fn deserialize_f32s(val: &[json::JsonValue]) -> Vec<f32> {
	val.iter().filter_map(JsonValue::as_f32).collect::<Vec<_>>()
}

fn deserialize_vec2s_flat(vals: &[json::JsonValue]) -> InoxParseResult<Vec<Vec2>> {
	if vals.len() % 2 != 0 {
		return Err(InoxParseError::OddNumberOfFloatsInList(vals.len()));
	}

	let floats = deserialize_f32s(vals);
	let mut vertices = Vec::new();
	vertices.extend_from_slice(f32s_as_vec2s(&floats));

	Ok(vertices)
}

fn deserialize_vec2(vals: &[json::JsonValue]) -> InoxParseResult<Vec2> {
	if vals.len() != 2 {
		return Err(InoxParseError::Not2FloatsInList(vals.len()));
	}

	let x = vals[0].as_f32().unwrap_or_default();
	let y = vals[1].as_f32().unwrap_or_default();
	Ok(vec2(x, y))
}

fn deserialize_vec2s(vals: &[json::JsonValue]) -> InoxParseResult<Vec<Vec2>> {
	let mut vec2s = Vec::with_capacity(vals.len());
	for (i, vals) in vals.iter().enumerate() {
		vec2s.push(deserialize_vec2(as_nested_list(i, vals)?)?);
	}
	Ok(vec2s)
}

// Puppet deserialization

impl Puppet {
	pub fn new_from_json(payload: &json::JsonValue) -> InoxParseResult<Self> {
		Self::new_from_json_with_custom(payload, None::<&fn(&mut Self, &str, JsonObject) -> InoxParseResult<()>>)
	}

	pub fn new_from_json_with_custom(
		payload: &json::JsonValue,
		load_node_data_custom: Option<&impl Fn(&mut Self, &str, JsonObject) -> InoxParseResult<()>>,
	) -> InoxParseResult<Self> {
		let obj = as_object("(puppet)", payload)?;

		let meta = vals("meta", deserialize_puppet_meta(obj.get_object("meta")?))?;
		let physics = vals("physics", deserialize_puppet_physics(obj.get_object("physics")?))?;
		let parameters = deserialize_params(obj.get_list("param")?)?;

		let root = vals("nodes", deserialize_node(obj.get_object("nodes")?))?;
		let ParsedNode {
			node,
			ty,
			data,
			children,
		} = root;
		let root_id = node.uuid;

		let mut puppet = Self::new(meta, physics, node, parameters);

		puppet.load_node_data(root_id, ty, data, load_node_data_custom)?;
		puppet.load_children_rec(root_id, children, load_node_data_custom)?;

		Ok(puppet)
	}

	fn load_node_data(
		&mut self,
		id: InoxNodeUuid,
		ty: &str,
		data: JsonObject,
		load_node_data_custom: Option<&impl Fn(&mut Self, &str, JsonObject) -> InoxParseResult<()>>,
	) -> InoxParseResult<()> {
		match ty {
			"Node" => (),
			"Part" => {
				self.node_comps.add(id, deserialize_drawable(data)?);
				self.node_comps.add(id, deserialize_textured_mesh(data)?);
				self.node_comps
					.add(id, vals("mesh", deserialize_mesh(data.get_object("mesh")?))?)
			}
			"Composite" => {
				self.node_comps.add(id, deserialize_drawable(data)?);
				self.node_comps.add(id, Composite {});
			}
			"SimplePhysics" => {
				self.node_comps.add(id, deserialize_simple_physics(data)?);
			}
			"MeshGroup" => {
				self.node_comps.add(id, deserialize_meshgroup(data)?);
				self.node_comps
					.add(id, vals("mesh", deserialize_mesh(data.get_object("mesh")?))?);
			}
			custom => {
				if let Some(func) = load_node_data_custom {
					func(self, custom, data)?
				}
			}
		}

		Ok(())
	}

	fn load_children_rec(
		&mut self,
		id: InoxNodeUuid,
		children: &[JsonValue],
		load_node_data_custom: Option<&impl Fn(&mut Self, &str, JsonObject) -> InoxParseResult<()>>,
	) -> InoxParseResult<()> {
		for (i, child) in children.iter().enumerate() {
			let msg = &format!("children[{}]", i);

			let child = as_object("child", child).map_err(|e| e.nested(msg))?;
			let child_node = deserialize_node(child).map_err(|e| e.nested(msg))?;
			let ParsedNode {
				node,
				ty,
				data,
				children,
			} = child_node;
			let child_id = node.uuid;

			self.nodes.add(id, child_id, node);
			self.load_node_data(child_id, ty, data, load_node_data_custom)
				.map_err(|e| e.nested(msg))?;
			if !children.is_empty() {
				self.load_children_rec(child_id, children, load_node_data_custom)
					.map_err(|e| e.nested(msg))?;
			}
		}

		Ok(())
	}
}

fn deserialize_params(vals: &[json::JsonValue]) -> InoxParseResult<HashMap<String, Param>> {
	let mut params = HashMap::new();

	for param in vals {
		let pair = deserialize_param(as_object("param", param)?)?;
		params.insert(pair.0, pair.1);
	}

	Ok(params)
}

fn deserialize_param(obj: JsonObject) -> InoxParseResult<(String, Param)> {
	let name = obj.get_str("name")?.to_owned();
	Ok((
		name.clone(),
		Param {
			uuid: ParamUuid(obj.get_u32("uuid")?),
			name,
			is_vec2: obj.get_bool("is_vec2")?,
			min: obj.get_vec2("min")?,
			max: obj.get_vec2("max")?,
			defaults: obj.get_vec2("defaults")?,
			axis_points: deserialize_axis_points(obj.get_list("axis_points")?)?,
			bindings: deserialize_bindings(obj.get_list("bindings")?)?,
		},
	))
}

fn deserialize_bindings(vals: &[json::JsonValue]) -> InoxParseResult<Vec<Binding>> {
	let mut bindings = Vec::new();
	for val in vals {
		let Ok(binding_object) = as_object("binding", val) else {
			tracing::error!("Encountered binding that is not a JSON object, ignoring");
			continue;
		};

		match deserialize_binding(binding_object) {
			Ok(binding) => bindings.push(binding),
			Err(e) => tracing::error!("Invalid binding: {e}"),
		}
	}

	Ok(bindings)
}

fn deserialize_binding(obj: JsonObject) -> InoxParseResult<Binding> {
	let is_set = obj
		.get_list("isSet")?
		.iter()
		.map(|bools| bools.members().map_while(JsonValue::as_bool).collect())
		.collect::<Vec<Vec<_>>>();

	Ok(Binding {
		node: InoxNodeUuid(obj.get_u32("node")?),
		is_set: Matrix2d::from_slice_vecs(&is_set, true)?,
		interpolate_mode: match obj.get_str("interpolate_mode")? {
			"Linear" => InterpolateMode::Linear,
			"Nearest" => InterpolateMode::Nearest,
			a => return Err(InoxParseError::UnknownInterpolateMode(a.to_owned())),
		},
		values: deserialize_binding_values(obj.get_str("param_name")?, obj.get_list("values")?)?,
	})
}

fn deserialize_binding_values(param_name: &str, values: &[JsonValue]) -> InoxParseResult<BindingValues> {
	Ok(match param_name {
		"zSort" => BindingValues::ZSort(deserialize_inner_binding_values(values)?),
		"transform.t.x" => BindingValues::TransformTX(deserialize_inner_binding_values(values)?),
		"transform.t.y" => BindingValues::TransformTY(deserialize_inner_binding_values(values)?),
		"transform.s.x" => BindingValues::TransformSX(deserialize_inner_binding_values(values)?),
		"transform.s.y" => BindingValues::TransformSY(deserialize_inner_binding_values(values)?),
		"transform.r.x" => BindingValues::TransformRX(deserialize_inner_binding_values(values)?),
		"transform.r.y" => BindingValues::TransformRY(deserialize_inner_binding_values(values)?),
		"transform.r.z" => BindingValues::TransformRZ(deserialize_inner_binding_values(values)?),
		"deform" => {
			let mut parsed = Vec::with_capacity(values.len());
			for (j, vals) in values.iter().enumerate() {
				let nested = as_nested_list(j, vals)?;
				let mut nested_parsed = Vec::with_capacity(nested.len());
				for (i, vals) in nested.iter().enumerate() {
					nested_parsed.push(deserialize_vec2s(as_nested_list(i, vals)?)?);
				}
				parsed.push(nested_parsed);
			}

			BindingValues::Deform(Matrix2d::from_slice_vecs(&parsed, true)?)
		}
		// TODO
		"opacity" => BindingValues::Opacity,
		param_name => return Err(InoxParseError::UnknownParamName(param_name.to_owned())),
	})
}

fn deserialize_inner_binding_values(values: &[JsonValue]) -> Result<Matrix2d<f32>, Matrix2dFromSliceVecsError> {
	let values = values
		.iter()
		.enumerate()
		.filter_map(|(i, vals)| Some(deserialize_f32s(as_nested_list(i, vals).ok()?)))
		.collect::<Vec<Vec<_>>>();

	Matrix2d::from_slice_vecs(&values, true)
}

fn deserialize_axis_points(vals: &[json::JsonValue]) -> InoxParseResult<AxisPoints> {
	let x = deserialize_f32s(as_nested_list(0, &vals[0])?);
	let y = deserialize_f32s(as_nested_list(1, &vals[1])?);
	Ok(AxisPoints { x, y })
}

fn deserialize_puppet_physics(obj: JsonObject) -> InoxParseResult<PuppetPhysics> {
	Ok(PuppetPhysics {
		pixels_per_meter: obj.get_f32("pixelsPerMeter")?,
		gravity: obj.get_f32("gravity")?,
	})
}

fn deserialize_puppet_meta(obj: JsonObject) -> InoxParseResult<PuppetMeta> {
	Ok(PuppetMeta {
		name: obj.get_nullable_str("name")?.map(str::to_owned),
		version: obj.get_str("version")?.to_owned(),
		rigger: obj.get_nullable_str("rigger")?.map(str::to_owned),
		artist: obj.get_nullable_str("artist")?.map(str::to_owned),
		rights: match obj.get_object("rights").ok() {
			Some(rights) => Some(deserialize_puppet_usage_rights(rights)?),
			None => None,
		},
		copyright: obj.get_nullable_str("copyright")?.map(str::to_owned),
		license_url: obj.get_nullable_str("licenseURL")?.map(str::to_owned),
		contact: obj.get_nullable_str("contact")?.map(str::to_owned),
		reference: obj.get_nullable_str("reference")?.map(str::to_owned),
		thumbnail_id: obj.get_u32("thumbnailId").ok(),
		preserve_pixels: obj.get_bool("preservePixels")?,
	})
}

fn deserialize_puppet_usage_rights(obj: JsonObject) -> InoxParseResult<PuppetUsageRights> {
	Ok(PuppetUsageRights {
		allowed_users: match obj.get_str("allowed_users")? {
			"OnlyAuthor" => PuppetAllowedUsers::OnlyAuthor,
			"OnlyLicensee" => PuppetAllowedUsers::OnlyLicensee,
			"Everyone" => PuppetAllowedUsers::Everyone,
			unknown => return Err(InoxParseError::UnknownPuppetAllowedUsers(unknown.to_owned())),
		},
		allow_violence: obj.get_bool("allow_violence")?,
		allow_sexual: obj.get_bool("allow_sexual")?,
		allow_commercial: obj.get_bool("allow_commercial")?,
		allow_redistribution: match obj.get_str("allow_redistribution")? {
			"Prohibited" => PuppetAllowedRedistribution::Prohibited,
			"ViralLicense" => PuppetAllowedRedistribution::ViralLicense,
			"CopyleftLicense" => PuppetAllowedRedistribution::CopyleftLicense,
			unknown => return Err(InoxParseError::UnknownPuppetAllowedRedistribution(unknown.to_owned())),
		},
		allow_modification: match obj.get_str("allow_modification")? {
			"Prohibited" => PuppetAllowedModification::Prohibited,
			"AllowPersonal" => PuppetAllowedModification::AllowPersonal,
			"AllowRedistribute" => PuppetAllowedModification::AllowRedistribute,
			unknown => return Err(InoxParseError::UnknownPuppetAllowedModification(unknown.to_owned())),
		},
		require_attribution: obj.get_bool("require_attribution")?,
	})
}
