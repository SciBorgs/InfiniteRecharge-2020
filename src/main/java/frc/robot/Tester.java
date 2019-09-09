package frc.robot;

public class Tester{

    private static void testFailed(String testName, String description){
        throw new RuntimeException("test \"" + testName + "\" FAILED: " + description);
    }
    private static void testBool(String testName, String description, boolean succeded){
        if (!succeded){testFailed(testName, description);}
    }

    public static void assertTrue(boolean b, String testName) {
        testBool(testName, "Statement false", b);
    }
    
    public static <N1 extends Number, N2 extends Number> void assertEquals(N1 n1, N2 n2, String testName) {
        double d1 = n1.doubleValue();
        double d2 = n2.doubleValue();
        testBool(testName, "Assertion that " + d1 + " == " + d2, d1 == d2);
    }

    public static void assertEquals(Object o1, Object o2, String testName){
        testBool(testName, "Assertion that " + o1 + " equals " + o2, o1.equals(o2));
    }

}