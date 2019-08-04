package frc.robot.helpers;

public class Matrix{

    // swap rows i and j
    private static void swap(double[][] matrix, int i, int j) {
        double[] temp = matrix[i];
        matrix[i] = matrix[j];
        matrix[j] = temp;
    }

    // create and return the transpose of the invoking matrix
    public static double[][] transpose(double[][] matrix) {
        double[][] temp = new double[matrix.length][matrix[0].length];
        for (int i = 0; i < matrix.length; i++) {
            for (int j = 0; j < matrix[i].length; j++) {
                temp[j][i] = matrix[i][j];
            }
        }
        return temp;
    }

    // return C = A + B
    public double[][] add(double[][] a, double[][] b) {
        if (b.length != a.length || b[0].length != a[0].length) throw new RuntimeException("Illegal matrix dimensions.");
        double[][] c = new double[b.length][b[0].length];
        for (int i = 0; i < b.length; i++) {
            for (int j = 0; j < b[0].length; j++) {
                c[i][j] = a[i][j] + b[i][j];
            }
        }
        return c;
    }

    // return C = A - B
    public double[][] subtract(double[][] a, double[][] b) {
        if (b.length != a.length || b[0].length != a[0].length) throw new RuntimeException("Illegal matrix dimensions.");
        double[][] c = new double[b.length][b[0].length];
        for (int i = 0; i < b.length; i++) {
            for (int j = 0; j < b[0].length; j++) {
                c[i][j] = a[i][j] - b[i][j];
            }
        }
        return c;
    }

    // return C = A * B
    public double[][] multiply(double[][] a, double[][] b) {
        if (b.length != a[0].length) throw new RuntimeException("Illegal matrix dimensions.");
        double[][] c = new double[b.length][b[0].length];
        
        for (int i = 0; i < b.length; i++) {
            for (int j = 0; j < b[0].length; j++) {
                for (int k = 0; k < a[0].length; k++) {
                    c[i][j] += (a[i][k] * b[k][j]);
                }
            }
        }
        return c;
    }

    // scale entire matrix by factor
    public static void multiply(double factor, double[][] a){
        for (int i = 0; i < a.length; i++) {
            for (int j = 0; j < a[0].length; j++) {
                a[i][j] *= factor;
            }
        }
    }
    
    // dot product of 2 vectors (can be 2D or 3D)
    public static double Dot(double[] a, double[] b) {
        if (b.length != a.length ) throw new RuntimeException("Illegal matrix dimensions.");
        double product = 0;
        for (int i = 0; i < b.length; i++) {
            product += a[i] * b[i];
        }
        return product;
    }


    // scale a specific row by a constant    
    public static void multiplyRow(double[][] a, int rowIndex , double scalar) {
		for (int k = rowIndex; k < a[rowIndex].length; k++) {
			a[rowIndex][k] *= scalar;
		}
    }
    
	public static void replaceRow(double[][] a, int j, int k) {
		double mult = -a[k][j];
		for (int i = j; i < a[k].length; i++) {
			a[k][i] += mult * a[j][i];
		}
	}
}