package frc.robot.helpers;

public class Matrix{
    
    public int rows, columns;
    private final double[][] matrix;
    
    // identity matrix
    public Matrix(int rows, int columns){ 
        this.rows = rows; 
        this.columns = columns;
        this.matrix = new double[this.rows][this.columns];
    }

    // new matrix based on 2D array
    public Matrix(double[][] matrix){
        rows = matrix.length;
        columns = matrix[0].length;
        this.matrix = new double[rows][columns];
        
        for(int i = 0; i < rows; i++){
            for(int j = 0; j < rows; j++){
                this.matrix[i][j] = matrix[i][j];
            }
        }
    }

    // copies existing Matrix
    public Matrix(Matrix matrix){ this(matrix.matrix); }

    public Matrix getMatrix(){ return this; }
    
    //get value of specific element
    public double getElement(int row, int column){
        return this.matrix[row][column];
    }

    // swap rows i and j
    private void swap(int i, int j) {
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
}