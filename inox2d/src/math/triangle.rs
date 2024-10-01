use std::num::NonZeroU16;

use glam::{Mat2, Vec2};

use crate::node::components::Mesh;

/// Undefined if point is exactly on the edge.
///
/// Though, due to floating point precision it is hard for a point to be exactly on the edge,
/// let alone that for points so close to the edge, whether they are actually in the triangle do not matter too much.
pub fn is_point_in_triangle(p: Vec2, triangle: &[Vec2; 3]) -> bool {
	#[inline]
	fn sign(p1: Vec2, p2: Vec2, p3: Vec2) -> f32 {
		Mat2::from_cols(p1, p2).sub_mat2(&Mat2::from_cols(p3, p3)).determinant()
	}

	let p1 = triangle[0];
	let p2 = triangle[1];
	let p3 = triangle[2];

	let d1 = sign(p, p1, p2);
	let d2 = sign(p, p2, p3);
	let d3 = sign(p, p3, p1);

	let has_neg = d1.is_sign_negative() || d2.is_sign_negative() || d3.is_sign_negative();
	let has_pos = d1.is_sign_positive() || d2.is_sign_positive() || d3.is_sign_positive();

	!(has_neg && has_pos)
}

/// Return top-left and bottom-right corners of the smallest covering rectangle over a list of points.
#[inline]
fn get_bounds<'a>(vertices: impl Iterator<Item = &'a Vec2>) -> (Vec2, Vec2) {
	let (mut x, mut y, mut z, mut w) = (f32::INFINITY, f32::INFINITY, f32::NEG_INFINITY, f32::NEG_INFINITY);
	vertices.for_each(|v| {
		(x, y, z, w) = (x.min(v.x), y.min(v.y), z.max(v.x), w.max(v.y));
	});
	(Vec2::new(x, y), Vec2::new(z, w))
}

impl Mesh {
	/// The `i`-th triangle as described by `self.indices`.
	pub fn get_triangle(&self, i: u16) -> [Vec2; 3] {
		[
			self.vertices[self.indices[3 * i as usize] as usize],
			self.vertices[self.indices[(3 * i + 1) as usize] as usize],
			self.vertices[self.indices[(3 * i + 2) as usize] as usize],
		]
	}

	/// Find which triangle of the mesh is a point in, if any, by brute force.
	pub fn test<'a>(&'a self, ps: impl Iterator<Item = &'a Vec2> + 'a) -> impl Iterator<Item = Option<u16>> + 'a {
		ps.map(|p| {
			(0..(self.indices.len() / 3) as u16).find(|&i| {
				let triangle = self.get_triangle(i);
				is_point_in_triangle(*p, &triangle)
			})
		})
	}
}

/// Cache for efficient mesh testing (which triangle is a point in?)
pub struct MeshBitMask<'mesh> {
	mesh: &'mesh Mesh,
	top_left: Vec2,
	// "Grid size" of the mask. The ref impl uses `1.0`.
	x_step: f32,
	y_step: f32,
	width: usize,
	height: usize,
	/// If `None`, the point belongs to no triangle.
	/// Else `Some(i)`, the point belongs to triangle made up of `mesh.indices[3*(i-1):3*i]`.
	///
	/// NOTE THE +1 in `i` here!
	mask: Vec<Option<NonZeroU16>>,
}

impl<'mesh> MeshBitMask<'mesh> {
	/// Find x coordinate of the nearest grid point on the left. Return boundary value for out-of-bounds input.
	#[inline]
	fn get_x(&self, x: f32) -> usize {
		if x <= self.top_left.x {
			0
		} else {
			((x - self.top_left.x) / self.x_step)
				.floor()
				.min((self.width - 1) as f32) as usize
		}
	}
	/// Find y coordinate of the nearest grid point on the top. Return boundary value for out-of-bounds input.
	#[inline]
	fn get_y(&self, y: f32) -> usize {
		if y <= self.top_left.y {
			0
		} else {
			((y - self.top_left.y) / self.y_step)
				.floor()
				.min((self.height - 1) as f32) as usize
		}
	}

	/// ONLY USE IN `new()`
	///
	/// Actually build `mask` content by testing grid points in regions spanned by each triangle.
	// Is a method with `&mut self` so that `get_x()` `get_y()` helpers could be reused.
	#[inline]
	fn build(&mut self) {
		self.mask.resize(self.width * self.height, None);

		for i in 0..(self.mesh.indices.len() / 3) as u16 {
			let vertices = self.mesh.get_triangle(i);

			let (region_top_left, region_bottom_right) = get_bounds(vertices.iter());
			let x_begin = self.get_x(region_top_left.x);
			let x_end = self.get_x(region_bottom_right.x);
			let y_begin = self.get_y(region_top_left.y);
			let y_end = self.get_y(region_bottom_right.y);

			for x in x_begin..=x_end {
				for y in y_begin..=y_end {
					let p = self.top_left + Vec2::new(x as f32 * self.x_step, y as f32 * self.y_step);
					if is_point_in_triangle(p, &vertices) {
						self.mask[x + y * self.width] = Some(NonZeroU16::new(i + 1).unwrap());
					}
				}
			}
		}
	}

	/// See `new()`.
	const MIN_STEP: f32 = 1.0;

	/// Create a `MeshBitMask` associated to `mesh`, storing a reference to it (thus living as long as `mesh`).
	pub fn new(mesh: &'mesh Mesh) -> Self {
		let (top_left, bottom_right) = get_bounds(mesh.vertices.iter());

		// TODO: Figure out if dynamic steps according to mesh are worthy, if so, how to properly do them.
		/*
		let mut x_step = f32::INFINITY;
		let mut y_step = f32::INFINITY;
		// If mesh is empty, step will keep being infinity, and this shall not cause problems anyways.
		for i in 0..(mesh.indices.len() / 3) as u16 {
			let [p0, p1, p2] = mesh.get_triangle(i);

			x_step = x_step
				.min(f32::abs(p0.x - p1.x))
				.min(f32::abs(p1.x - p2.x))
				.min(f32::abs(p2.x - p0.x));
			y_step = y_step
				.min(f32::abs(p0.y - p1.y))
				.min(f32::abs(p1.y - p2.y))
				.min(f32::abs(p2.y - p0.y));

			// to prevent step getting too small when badly shaped triangles present
			// Yes, this would yield mathematically wrong results if mesh too small, but it is the rigger's problem that they are creating sub-pixel meshes.
			if x_step < Self::MIN_STEP {
				x_step = Self::MIN_STEP;
				tracing::warn!(
					"A triangle is too thin on x direction. Testing with this MeshBitMask may not be accurate."
				);
				break;
			}
			if y_step < Self::MIN_STEP {
				y_step = Self::MIN_STEP;
				tracing::warn!(
					"A triangle is too thin on y direction. Testing with this MeshBitMask may not be accurate."
				);
				break;
			}
		}
		*/
		let x_step = Self::MIN_STEP;
		let y_step = Self::MIN_STEP;

		let width = ((bottom_right.x - top_left.x) / x_step).ceil() as usize;
		let height = ((bottom_right.y - top_left.y) / y_step).ceil() as usize;

		let mut this = Self {
			mesh,
			top_left,
			x_step,
			y_step,
			width,
			height,
			mask: Vec::new(),
		};
		this.build();
		this
	}

	/// Return the index of the triangle point `p` is in, if any.
	pub fn test(&self, p: Vec2) -> Option<u16> {
		// handle empty mesh case
		if self.mask.is_empty() {
			return None;
		}

		let mut candidates = Vec::with_capacity(9);
		for dx in [-self.x_step, 0.0, self.x_step] {
			for dy in [-self.y_step, 0.0, self.y_step] {
				// x/y out of bounds are already handled in the getters.
				if let Some(index_plus_one) = self.mask[self.get_x(p.x + dx) + self.get_y(p.y + dy) * self.width] {
					candidates.push(index_plus_one.get() - 1);
				}
			}
		}
		candidates.dedup();

		candidates
			.into_iter()
			.find(|&t| is_point_in_triangle(p, &self.mesh.get_triangle(t)))
	}
}

#[cfg(test)]
mod tests {
	use std::f32::consts::PI;

	use glam::{vec2, Affine2};

	use super::*;

	/// Run the test function with arbitary affine transforms, which should not change certain properties (e.g. triangle test).
	fn test_with_affine(f: impl Fn(&Affine2)) {
		for scale in [vec2(1.0, 1.0), vec2(1.0, 3.0), vec2(3.0, 1.0), vec2(7.0, 10.0)] {
			for angle in [0.0, PI / 3.0, -PI / 2.0, PI] {
				for translation in [vec2(0.0, 0.0), vec2(1.0, 2.0), vec2(-2.0, 1.0), vec2(-2.0, -2.0)] {
					let transform = Affine2::from_scale_angle_translation(scale, angle, translation);
					f(&transform);
				}
			}
		}
	}

	/// Run the test function with mesh(es) and test points and answers, under the given affine transform.
	fn test_with_mesh(transform: Affine2, f: impl Fn(&Mesh, Vec<Vec2>) -> Vec<Option<u16>>) {
		let vertices = vec![
			vec2(2.0, 0.0),
			vec2(0.0, 8.0),
			vec2(4.0, 2.0),
			vec2(8.0, 6.0),
			vec2(10.0, 4.0),
		]
		.into_iter()
		.map(|v| transform.transform_point2(v))
		.collect();
		let indices = vec![0, 1, 2, 0, 2, 4, 1, 2, 3, 2, 3, 4];
		// Invalid mesh struct as `uvs`` is empty, but does not affect testing.
		let mesh = Mesh {
			vertices,
			uvs: Vec::new(),
			indices,
			origin: Vec2::ZERO,
		};

		let points_and_ans: [(Vec2, Option<u16>); 13] = [
			(vec2(-1.0, 0.0), None),
			(vec2(5.0, 1.0), None),
			(vec2(9.0, 6.0), None),
			(vec2(5.0, 7.0), None),
			(vec2(100.0, 100.0), None),
			(vec2(2.0, 2.0), Some(0)),
			(vec2(5.0, 2.0), Some(1)),
			(vec2(3.0, 2.0), Some(0)),
			(vec2(3.0, 3.0), Some(0)),
			(vec2(4.0, 3.0), Some(2)),
			(vec2(6.0, 3.0), Some(3)),
			(vec2(4.0, 6.0), Some(2)),
			(vec2(8.0, 5.0), Some(3)),
		];
		let points: Vec<Vec2> = points_and_ans.iter().map(|p| transform.transform_point2(p.0)).collect();
		let ans: Vec<Option<u16>> = points_and_ans.iter().map(|p| p.1).collect();

		assert_eq!(f(&mesh, points), ans);
	}

	#[test]
	fn bounds() {
		let vertices = [vec2(0.0, 1.0), vec2(1.0, 0.0), vec2(-1.0, -1.0)];
		assert_eq!(get_bounds(vertices.iter()), (glam::vec2(-1.0, -1.0), vec2(1.0, 1.0)))
	}

	#[test]
	fn triangle() {
		let t0 = vec2(0.5, 1.0);
		let t1 = vec2(1.0, 0.5);
		let t2 = vec2(0.0, 0.0);
		let test_xs = Vec::from_iter((0..=3).map(|i| (i as f32) / 3.0));
		let test_ys = Vec::from_iter((1..=5).map(|i| (i as f32) / 5.0));
		let ans = [
			[false, true, false, false],
			[false, true, true, false],
			[false, true, true, false],
			[false, false, true, false],
			[false, false, false, false],
		];

		test_with_affine(|transform| {
			let t0 = transform.transform_point2(t0);
			let t1 = transform.transform_point2(t1);
			let t2 = transform.transform_point2(t2);
			let triangle = [t0, t1, t2];

			for (iy, y) in test_ys.iter().enumerate() {
				for (ix, x) in test_xs.iter().enumerate() {
					let p = transform.transform_point2(vec2(*x, *y));
					assert_eq!(
						is_point_in_triangle(p, &triangle),
						ans[iy][ix],
						"Triangle: [{t0}, {t1}, {t2}], point: {p}",
					);
				}
			}
		});
	}

	#[test]
	fn mesh_test() {
		test_with_affine(|transform| test_with_mesh(*transform, |mesh, ps| mesh.test(ps.iter()).collect()))
	}

	#[test]
	fn bit_mask() {
		test_with_affine(|transform| {
			test_with_mesh(*transform, |mesh, ps| {
				let bit_mask = MeshBitMask::new(mesh);

				ps.into_iter().map(|p| bit_mask.test(p)).collect()
			})
		})
	}

	#[test]
	fn bit_mask_empty_mesh() {
		let mesh = Mesh {
			vertices: Vec::new(),
			uvs: Vec::new(),
			indices: Vec::new(),
			origin: Vec2::ZERO,
		};
		let bit_mask = MeshBitMask::new(&mesh);

		assert_eq!(bit_mask.width, 0);
		assert_eq!(bit_mask.height, 0);
		assert_eq!(bit_mask.test(vec2(-1.0, 0.0)), None);
		assert_eq!(bit_mask.test(vec2(1.0, 2.0)), None);
	}
}
