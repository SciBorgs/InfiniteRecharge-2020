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
    
    public static void assertEquals(Number n1, Number n2, String testName){
        testBool(testName, "Assertion that " + n1 + " == " + n2, n1 == n2);
    }

    public static void assertEquals(Object o1, Object o2, String testName){
        testBool(testName, "Assertion that " + o1 + " equals " + o2, o1.equals(o2));
    }

}