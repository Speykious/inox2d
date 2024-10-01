use glam::{Mat2, Vec2};

/// Different kinds of deform.
// TODO: Meshgroup.
pub(crate) enum Deform {
	/// Specifying a displacement for every vertex.
	Direct(Vec<Vec2>),
}

/// Element-wise add direct deforms up and write result.
pub(crate) fn linear_combine<'deforms>(direct_deforms: impl Iterator<Item = &'deforms Vec<Vec2>>, result: &mut [Vec2]) {
	result.iter_mut().for_each(|deform| *deform = Vec2::ZERO);

	for direct_deform in direct_deforms {
		if direct_deform.len() != result.len() {
			panic!("Trying to combine direct deformations with wrong dimensions.");
		}

		result
			.iter_mut()
			.zip(direct_deform.iter())
			.for_each(|(sum, addition)| *sum += *addition);
	}
}

/// Input: two basis vectors `b0` and `b1`.
///
/// If the returned matrix is to be applied on a Vec2 V and X is obtained,
/// then `X.x * b0 + X.y * b1 = V`.
///
/// Panics if either basis is zero or they are not independent of each other.
pub fn vector_decompose_matrix(b0: Vec2, b1: Vec2) -> Mat2 {
	// B X = V where:
	// B: [ b0.x b1.x
	//      b0.y b1.y ]
	// X: [ x
	//      y ]
	// V: [ v.x
	//      v.y ]
	// thus X = B^-1 V
	let mat = Mat2::from_cols(b0, b1).inverse();
	debug_assert_ne!(mat.determinant(), 0.0, "Provided two basis do not span the 2D plane.");
	mat
}

/// Provide a parent triangle and its deforms by 3 points,
/// calculate how far should the provided points be moved by the triangle's deform.
///
/// For optimization, the "decompose_matrix" of parent should be provided, see `vector_decompose_matrix()`.
/// It is assumed that `parent[0]` is taken as the origin,
/// `parent[1] - parent[0]` is the first basis vector, and `parent[2] - parent[0]` the second.
#[inline]
pub fn deform_by_parent_triangle<'a>(
	decompose_matrix: &'a Mat2,
	parent_p0: Vec2,
	parent_deforms: &'a [Vec2; 3],
	points: impl Iterator<Item = &'a Vec2> + 'a,
) -> impl Iterator<Item = Vec2> + 'a {
	let basis_0_deform = parent_deforms[1] - parent_deforms[0];
	let basis_1_deform = parent_deforms[2] - parent_deforms[0];

	points.map(move |p| {
		let decomposed_coeffs = *decompose_matrix * (*p - parent_p0);
		// deform by parent[0] + deform by basis change
		parent_deforms[0] + decomposed_coeffs.x * basis_0_deform + decomposed_coeffs.y * basis_1_deform
	})
}
