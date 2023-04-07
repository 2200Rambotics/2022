package frc.robot;

public class LinearInterpolationTest {

    public static void main(String[] args) {
        LinearInterpolationTest t = new LinearInterpolationTest();
        t.tableTest();
    }

    public void tableTest() {
        double[] [] values = {
            {0,0},
            {1,5},
            {2,8},
            {3,10}
        };
        LinearInterpolation l = new LinearInterpolation(values);

        double[][] test_cases = {
            {0.5, 2.5},
            {1, 5},
            {0, 0},
            {-1, -5},
            {4, 12},
            {0.75, 3.75}
        };

        double[][] test_cases2 = {
            {1, 3.768},
            {2, 8.321},
            {3, 13.659},
            {4, 19.782},
            {5, 26.69}
        };

        for (int i = 0; i < test_cases.length; i++) {
            double input = test_cases[i][0];
            double expected = test_cases[i][1];
            double actual = l.interpolate(input);
            if (expected != actual) {
                System.out.printf("ERROR interpolate(%f) should be %f, was %f\n", input, expected, actual);
            } else {
                System.out.println("PASS");
            }  
        }
    }
}
