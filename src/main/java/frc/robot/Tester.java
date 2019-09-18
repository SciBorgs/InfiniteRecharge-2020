package frc.robot;

public class Tester{

    private static void testFailed(String testName, String message){
        throw new RuntimeException("test \"" + testName + "\" FAILED: " + message);
    }
    private static void testBool(String testName, String message, boolean succeded){
        if (!succeded){testFailed(testName, message);}
    }

    public static void assertTrue(boolean b, String testName) {
        testBool(testName, "Statement false", b);
    }
    public static void assertFalse(boolean b, String testName) {
        testBool(testName, "Statement true", !b);
    }
    
    private static <N1 extends Number, N2 extends Number> void equalAssertion(N1 n1, N2 n2, String testName, boolean requireEqual) {
        // It is possible you could get some rounding errors here, maybe not
        // Someone should look into that
        double d1 = n1.doubleValue();
        double d2 = n2.doubleValue();
        String equalString = requireEqual ? "==" : "!=";
        testBool(testName, "Assertion that " + d1 + " " + equalString + " " + d2, d1 != d2 ^ requireEqual);
    }

    public static void assertEquals(Object o1, Object o2, String testName){
        testBool(testName, "Assertion that " + o1 + " equals " + o2, o1.equals(o2));
    }
    public static void assertNotEquals(Object o1, Object o2, String testName) {
        testBool(testName, "Assertion that " + o1 + " doesn't equal " + o2, !o1.equals(o2));
    }

    public static <N1 extends Number, N2 extends Number> void assertEquals(N1 n1, N2 n2, String testName) {
        equalAssertion(n1, n2, testName, true);
    }
    public static <N1 extends Number, N2 extends Number> void assertNotEquals(N1 n1, N2 n2, String testName) {
        equalAssertion(n1, n2, testName, false);
    }

}