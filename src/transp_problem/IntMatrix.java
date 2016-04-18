package transp_problem;

public class IntMatrix {

	private int[][] A;
	private int m, n;

	public IntMatrix(int m, int n) {
		this.m = m;
		this.n = n;
		A = new int[m][n];
	}

	public IntMatrix(int m, int n, int value) {
		this.m = m;
		this.n = n;
		A = new int[m][n];
		for (int i = 0; i < m; i++) {
			for (int j = 0; j < n; j++) {
				A[i][j] = value;
			}
		}
	}

	public IntMatrix(int[][] A) {
		m = A.length;
		n = A[0].length;
		for (int i = 0; i < m; i++) {
			if (A[i].length != n) {
				throw new IllegalArgumentException("All rows must have the same length.");
			}
		}
		this.A = A;
	}

	/**
	 * Construct a matrix from a one-dimensional packed array
	 * 
	 * @param values One-dimensional array of int, packed by columns (ala Fortran).
	 * @param m Number of rows.
	 * @exception IllegalArgumentException Array length must be a multiple of m.
	 */
	public IntMatrix(int values[], int m) {
		this.m = m;
		n = (m != 0 ? values.length / m : 0);
		if (m * n != values.length) {
			throw new IllegalArgumentException("Array length must be a multiple of m.");
		}
		A = new int[m][n];
		for (int i = 0; i < m; i++) {
			for (int j = 0; j < n; j++) {
				A[i][j] = values[i + j * m];
			}
		}
	}

	/*
	 * ------------------------ Public Methods ------------------------
	 */

	/**
	 * Construct a matrix from a copy of a 2-D array.
	 * 
	 * @param A Two-dimensional array of doubles.
	 * @exception IllegalArgumentException All rows must have the same length
	 */

	public static IntMatrix constructWithCopy(int[][] A) {
		int m = A.length;
		int n = A[0].length;
		IntMatrix X = new IntMatrix(m, n);
		int[][] C = X.getArray();
		for (int i = 0; i < m; i++) {
			if (A[i].length != n) {
				throw new IllegalArgumentException("All rows must have the same length.");
			}
			for (int j = 0; j < n; j++) {
				C[i][j] = A[i][j];
			}
		}
		return X;
	}

	/**
	 * Make a deep copy of a matrix
	 */

	public IntMatrix copy() {
		IntMatrix X = new IntMatrix(m, n);
		int[][] C = X.getArray();
		for (int i = 0; i < m; i++) {
			for (int j = 0; j < n; j++) {
				C[i][j] = A[i][j];
			}
		}
		return X;
	}

	/**
	 * Clone the Matrix object.
	 */

	public Object clone() {
		return this.copy();
	}

	public int[][] getArray() {
		return A;
	}

	public int[][] getArrayCopy() {
		int[][] C = new int[m][n];
		for (int i = 0; i < m; i++) {
			for (int j = 0; j < n; j++) {
				C[i][j] = A[i][j];
			}
		}
		return C;
	}

	public int[] getColumnPackedCopy() {
		int[] vals = new int[m * n];
		for (int i = 0; i < m; i++) {
			for (int j = 0; j < n; j++) {
				vals[i + j * m] = A[i][j];
			}
		}
		return vals;
	}

	public int[] getRowPackedCopy() {
		int[] vals = new int[m * n];
		for (int i = 0; i < m; i++) {
			for (int j = 0; j < n; j++) {
				vals[i * n + j] = A[i][j];
			}
		}
		return vals;
	}

	public int getRowDimension() {
		return m;
	}

	public int getColumnDimension() {
		return n;
	}

	public int get(int i, int j) {
		return A[i][j];
	}

	/**
	 * Get a submatrix.
	 * 
	 * @param i0 Initial row index
	 * @param i1 Final row index
	 * @param j0 Initial column index
	 * @param j1 Final column index
	 * @return A(i0:i1,j0:j1)
	 * @exception ArrayIndexOutOfBoundsException Submatrix indices
	 */

	public IntMatrix getSubMatrix(int i0, int i1, int j0, int j1) {
		IntMatrix X = new IntMatrix(i1 - i0 + 1, j1 - j0 + 1);
		int[][] B = X.getArray();
		try {
			for (int i = i0; i <= i1; i++) {
				for (int j = j0; j <= j1; j++) {
					B[i - i0][j - j0] = A[i][j];
				}
			}
		} catch (ArrayIndexOutOfBoundsException e) {
			throw new ArrayIndexOutOfBoundsException("Submatrix indices");
		}
		return X;
	}

	/**
	 * Get a submatrix.
	 * 
	 * @param rows Array of row indices.
	 * @param columns Array of column indices.
	 * @return A(r(:),c(:))
	 * @exception ArrayIndexOutOfBoundsException Submatrix indices
	 */

	public IntMatrix getSubMatrix(int[] rows, int[] columns) {
		IntMatrix X = new IntMatrix(rows.length, columns.length);
		int[][] B = X.getArray();
		try {
			for (int i = 0; i < rows.length; i++) {
				for (int j = 0; j < columns.length; j++) {
					B[i][j] = A[rows[i]][columns[j]];
				}
			}
		} catch (ArrayIndexOutOfBoundsException e) {
			throw new ArrayIndexOutOfBoundsException("Submatrix indices");
		}
		return X;
	}

	/**
	 * Get a submatrix.
	 * 
	 * @param i0 Initial row index
	 * @param i1 Final row index
	 * @param columns Array of column indices.
	 * @return A(i0:i1,c(:))
	 * @exception ArrayIndexOutOfBoundsException Submatrix indices
	 */

	public IntMatrix getSubMatrix(int i0, int i1, int[] columns) {
		IntMatrix X = new IntMatrix(i1 - i0 + 1, columns.length);
		int[][] B = X.getArray();
		try {
			for (int i = i0; i <= i1; i++) {
				for (int j = 0; j < columns.length; j++) {
					B[i - i0][j] = A[i][columns[j]];
				}
			}
		} catch (ArrayIndexOutOfBoundsException e) {
			throw new ArrayIndexOutOfBoundsException("Submatrix indices");
		}
		return X;
	}

	/**
	 * Get a submatrix.
	 * 
	 * @param rows Array of row indices.
	 * @param i0 Initial column index
	 * @param i1 Final column index
	 * @return A(r(:),j0:j1)
	 * @exception ArrayIndexOutOfBoundsException Submatrix indices
	 */

	public IntMatrix getSubMatrix(int[] rows, int j0, int j1) {
		IntMatrix X = new IntMatrix(rows.length, j1 - j0 + 1);
		int[][] B = X.getArray();
		try {
			for (int i = 0; i < rows.length; i++) {
				for (int j = j0; j <= j1; j++) {
					B[i][j - j0] = A[rows[i]][j];
				}
			}
		} catch (ArrayIndexOutOfBoundsException e) {
			throw new ArrayIndexOutOfBoundsException("Submatrix indices");
		}
		return X;
	}

	public void set(int i, int j, int value) {
		A[i][j] = value;
	}

	/**
	 * Set a submatrix.
	 * 
	 * @param i0 Initial row index
	 * @param i1 Final row index
	 * @param j0 Initial column index
	 * @param j1 Final column index
	 * @param X A(i0:i1,j0:j1)
	 * @exception ArrayIndexOutOfBoundsException Submatrix indices
	 */

	public void setSubMatrix(int i0, int i1, int j0, int j1, IntMatrix X) {
		try {
			for (int i = i0; i <= i1; i++) {
				for (int j = j0; j <= j1; j++) {
					A[i][j] = X.get(i - i0, j - j0);
				}
			}
		} catch (ArrayIndexOutOfBoundsException e) {
			throw new ArrayIndexOutOfBoundsException("Submatrix indices");
		}
	}

	/**
	 * Set a submatrix.
	 * 
	 * @param rows Array of row indices.
	 * @param columns Array of column indices.
	 * @param X A(r(:),c(:))
	 * @exception ArrayIndexOutOfBoundsException Submatrix indices
	 */

	public void setSubMatrix(int[] rows, int[] columns, IntMatrix X) {
		try {
			for (int i = 0; i < rows.length; i++) {
				for (int j = 0; j < columns.length; j++) {
					A[rows[i]][columns[j]] = X.get(i, j);
				}
			}
		} catch (ArrayIndexOutOfBoundsException e) {
			throw new ArrayIndexOutOfBoundsException("Submatrix indices");
		}
	}

	/**
	 * Set a submatrix.
	 * 
	 * @param rows Array of row indices.
	 * @param j0 Initial column index
	 * @param j1 Final column index
	 * @param X A(r(:),j0:j1)
	 * @exception ArrayIndexOutOfBoundsException Submatrix indices
	 */

	public void setSubMatrix(int[] rows, int j0, int j1, IntMatrix X) {
		try {
			for (int i = 0; i < rows.length; i++) {
				for (int j = j0; j <= j1; j++) {
					A[rows[i]][j] = X.get(i, j - j0);
				}
			}
		} catch (ArrayIndexOutOfBoundsException e) {
			throw new ArrayIndexOutOfBoundsException("Submatrix indices");
		}
	}

	/**
	 * Set a submatrix.
	 * 
	 * @param i0 Initial row index
	 * @param i1 Final row index
	 * @param columns Array of column indices.
	 * @param X A(i0:i1,c(:))
	 * @exception ArrayIndexOutOfBoundsException Submatrix indices
	 */

	public void setSubMatrix(int i0, int i1, int[] columns, IntMatrix X) {
		try {
			for (int i = i0; i <= i1; i++) {
				for (int j = 0; j < columns.length; j++) {
					A[i][columns[j]] = X.get(i - i0, j);
				}
			}
		} catch (ArrayIndexOutOfBoundsException e) {
			throw new ArrayIndexOutOfBoundsException("Submatrix indices");
		}
	}

	public IntMatrix transpose() {
		IntMatrix X = new IntMatrix(n, m);
		int[][] C = X.getArray();
		for (int i = 0; i < m; i++) {
			for (int j = 0; j < n; j++) {
				C[j][i] = A[i][j];
			}
		}
		return X;
	}

	/**
	 * Generate matrix with random elements
	 * 
	 * @param m Number of rows.
	 * @param n Number of colums.
	 * @return An m-by-n matrix with uniformly distributed random elements.
	 */

	public static IntMatrix random(int m, int n) {
		IntMatrix A = new IntMatrix(m, n);
		int[][] X = A.getArray();
		for (int i = 0; i < m; i++) {
			for (int j = 0; j < n; j++) {
				X[i][j] = (int) Math.random();
			}
		}
		return A;
	}

	/**
	 * Generate identity matrix
	 * 
	 * @param m Number of rows.
	 * @param n Number of colums.
	 * @return An m-by-n matrix with ones on the diagonal and zeros elsewhere.
	 */

	public static IntMatrix identity(int m, int n) {
		IntMatrix A = new IntMatrix(m, n);
		int[][] X = A.getArray();
		for (int i = 0; i < m; i++) {
			for (int j = 0; j < n; j++) {
				X[i][j] = (i == j ? 1 : 0);
			}
		}
		return A;
	}

	public void print() {
		System.out.println("");
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 4; j++)
				System.out.print(this.get(i, j));
			System.out.println("");
		}
	}
}
