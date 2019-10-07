package frc.robot.helpers;

public class Matrix{

    public int rows, columns;
    public double[][] matrix;

    // identity matrix
    public Matrix(int rows, int columns){ 
        this.rows = rows; 
        this.columns = columns;
        this.matrix = new double[this.rows][this.columns];
    }

    // new matrix based on 2D array
    public Matrix(double[][] matrix) {
        checkDimensions(matrix);

        rows = matrix.length;
        columns = matrix[0].length;
        this.matrix = new double[rows][columns];

        for(int i = 0; i < rows; i++){
            for(int j = 0; j < columns; j++){
                this.matrix[i][j] = matrix[i][j];
            }
        }
    }

    // makes sure number of elements in a row are the same for each row
    public static void checkDimensions(double[][] matrix) {
        int numCol1, numCol2;
        numCol1 = matrix[0].length;
        for(int i = 1; i < matrix.length  - 1; i++) {
            numCol2 = matrix[i].length;
            if (!(numCol1 == numCol2)) throw new RuntimeException("Illegal matrix dimensions.");
        }
    }

    // copies existing Matrix
    public Matrix(Matrix matrix){ this(matrix.matrix); }

    public Matrix getMatrix(){ return this; }

    //get value of specific element
    public double getElement(int row, int column) {
        return this.matrix[row][column];
    }

    // swap rows i and j
    public void swap(int i, int j) {
        double[] temp = matrix[i];
        matrix[i] = matrix[j];
        matrix[j] = temp;
    }

    // create and return the transpose of the invoking matrix
    public Matrix transpose() {
        Matrix A = new Matrix(columns, rows);
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < columns; j++)
                A.matrix[j][i] = this.matrix[i][j];
        return A;
    }

    // return C = A + B
    public Matrix add(Matrix B) {
        Matrix A = this.getMatrix();
        if (B.rows != A.rows || B.columns != A.columns) throw new RuntimeException("Illegal matrix dimensions.");
        Matrix C = new Matrix(rows, columns);
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < columns; j++)
                C.matrix[i][j] = A.matrix[i][j] + B.matrix[i][j];
        return C;
    }

    // return C = A - B
    public Matrix subtract(Matrix B) {
        Matrix A = this.getMatrix();
        if (B.rows != A.rows || B.columns != A.columns) throw new RuntimeException("Illegal matrix dimensions.");
        Matrix C = new Matrix(rows, columns);
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < columns; j++)
                C.matrix[i][j] = A.matrix[i][j] - B.matrix[i][j];
        return C;
    }

    // does A = B exactly?
    public boolean equals(Matrix B) {
        Matrix A = this.getMatrix();
        if (B.rows != A.rows || B.columns != A.columns) throw new RuntimeException("Illegal matrix dimensions.");
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < columns; j++)
                if (A.matrix[i][j] != B.matrix[i][j]) return false;
        return true;
    }

    // return C = A * B
    public Matrix multiply(Matrix B) {
        Matrix A = this.getMatrix();
        if (A.columns != B.rows) throw new RuntimeException("Illegal matrix dimensions.");
        Matrix C = new Matrix(A.rows, B.columns);
        for (int i = 0; i < C.rows; i++)
            for (int j = 0; j < C.columns; j++)
                for (int k = 0; k < A.columns; k++)
                    C.matrix[i][j] += (A.matrix[i][k] * B.matrix[k][j]);
        return C;
    }

    // scale a specific row by a constant    
    void multiplyRow(Matrix A, int rowIndex , double scalar) {
		for (int k = rowIndex; k < A.matrix[rowIndex].length; k++) {
			A.matrix[rowIndex][k] *= scalar;
		}
	}

	void replaceRow(Matrix A, int j, int k) {
		double mult = -A.matrix[k][j];
		for (int i = j; i < A.matrix[k].length; i++) {
			A.matrix[k][i] += mult * A.matrix[j][i];
		}
	}

    // scale entire matrix by factor
    public static void multiply(Matrix A, double scaleFactor) {
        for (int i = 0; i < A.rows; i++) {
            for (int j = 0; j < A.columns; j++) {
                A.matrix[i][j] *= scaleFactor;
            }
        }
    }
}