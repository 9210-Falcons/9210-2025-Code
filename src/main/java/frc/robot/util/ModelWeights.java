package frc.robot.util;

public class ModelWeights {
  public static double[][] weights1 = {
    {
      -0.19310856,
      -0.4167877,
      0.24998216,
      -0.3845762,
      0.03238722,
      -0.03127768,
      -0.57876897,
      -0.02169357,
      0.24765687,
      -0.08676967,
      0.06757506,
      -0.27869877,
      -0.14129698,
      -0.41891018,
      0.6397747,
      0.13805671
    },
    {
      -0.45089304,
      -0.37370345,
      0.401669,
      -0.4297322,
      -0.01932225,
      0.15112855,
      0.17707054,
      -0.04465846,
      0.3835776,
      0.16725174,
      -0.42141002,
      0.16863368,
      -0.09181017,
      -0.32896268,
      -0.3977518,
      -0.2008094
    },
    {
      0.31485295,
      -0.13027823,
      -0.09056859,
      0.21075273,
      0.02360881,
      0.7157307,
      0.14966245,
      0.19521737,
      0.2050722,
      -0.19846113,
      0.57929546,
      -0.3311025,
      0.42474392,
      -0.35071114,
      0.00586517,
      0.46457866
    },
    {
      0.41976464,
      -0.23967993,
      -0.14164583,
      -0.42327845,
      -0.17556888,
      0.5475014,
      0.58638144,
      0.57742155,
      0.16843252,
      0.10462862,
      0.26472685,
      0.31918713,
      0.30195418,
      -0.34493408,
      -0.09410029,
      -0.37263802
    },
    {
      -0.43889868,
      0.6560941,
      -0.15022688,
      -0.1281935,
      -0.519742,
      -0.30802068,
      0.1613856,
      -0.1451901,
      -0.15053962,
      0.19743699,
      -0.01688145,
      0.20589732,
      -0.2595968,
      -0.15254791,
      -0.85796195,
      -0.07980584
    },
    {
      -0.4669485,
      0.17042919,
      -0.80684316,
      -0.20995033,
      0.3259983,
      0.12996358,
      0.4475006,
      0.31848818,
      0.33011952,
      0.41583845,
      0.12252724,
      -0.1619018,
      0.06342027,
      0.00615925,
      0.47782746,
      0.37389073
    },
    {
      -0.39054954,
      -0.20761476,
      -0.21085279,
      -0.35298955,
      0.37218747,
      0.05927853,
      -0.23144716,
      -0.12525597,
      -0.29827246,
      -0.7228868,
      0.0184443,
      -0.2694433,
      -0.45139623,
      0.03268357,
      0.00563203,
      0.15864299
    },
    {
      -0.49823654,
      0.21989661,
      0.5780565,
      -0.00625896,
      -0.4780869,
      -0.5131824,
      -0.08699852,
      -0.8457416,
      0.3503184,
      -0.46307117,
      0.33653516,
      -0.2890844,
      0.64471257,
      0.21071908,
      -0.09133472,
      -0.20137668
    }
  };

  public static double[][] biases1 = {
    {
      0.,
      0.299474,
      0.05551823,
      0.,
      0.2688127,
      0.14870794,
      0.21799423,
      0.06791857,
      -0.15571599,
      0.15786706,
      0.0186776,
      -0.12959898,
      0.06305254,
      -0.01033889,
      0.23794435,
      -0.05053543
    }
  };

  public static double[][] weights2 = {
    {-0.35580325, 0.07875001},
    {0.4295822, 0.48109937},
    {0.45442793, 1.0676788},
    {0.35006446, -0.32477522},
    {-0.61368155, 0.5069666},
    {0.52897656, 0.46830896},
    {0.41394687, 0.42290214},
    {-0.71720487, -0.9934867},
    {-0.08848078, -0.01607242},
    {-0.35278383, 0.5910301},
    {-0.21846215, 0.48030633},
    {0.02015607, -0.43299827},
    {-0.4943555, -0.8439114},
    {0.527406, -0.07437825},
    {0.7419444, 0.5974771},
    {0.0899217, -0.44715825}
  };
  public static double[][] biases2 = {{0.01303051, 0.07318439}};

  public static double[][] matrixMultiply(double[][] A, double[][] B) {
    int rowsA = A.length, colsA = A[0].length;
    int rowsB = B.length, colsB = B[0].length;

    if (colsA != rowsB) {
      throw new IllegalArgumentException("Matrix dimensions do not match for multiplication.");
    }

    double[][] result = new double[rowsA][colsB];

    for (int i = 0; i < rowsA; i++) {
      for (int j = 0; j < colsB; j++) {
        for (int k = 0; k < colsA; k++) {
          result[i][j] += A[i][k] * B[k][j];
        }
      }
    }

    return result;
  }

  public static double[][] matrixAdd(double[][] A, double[][] B) {
    int rows = A.length, cols = A[0].length;

    if (rows != B.length || cols != B[0].length) {
      throw new IllegalArgumentException("Matrix dimensions do not match for addition.");
    }

    double[][] result = new double[rows][cols];

    for (int i = 0; i < rows; i++) {
      for (int j = 0; j < cols; j++) {
        result[i][j] = A[i][j] + B[i][j];
      }
    }

    return result;
  }

  public static double[][] applyReLU(double[][] matrix) {
    int rows = matrix.length, cols = matrix[0].length;
    double[][] result = new double[rows][cols];

    for (int i = 0; i < rows; i++) {
      for (int j = 0; j < cols; j++) {
        result[i][j] = Math.max(0, matrix[i][j]);
      }
    }

    return result;
  }
}
