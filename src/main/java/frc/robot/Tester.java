package frc.robot;

public class Tester{

    public static void assertTrue(boolean b, String testName) {
        if (!b) {
            throw new RuntimeException("test \"" + testName + "\" FAILED: Statement false");
        }
    }
    
    public static void assertEquals(Number n1, Number n2, String testName){
        if (n1 != n2){
            throw new RuntimeException("test \"" + testName + "\" FAILED: Assertion that " + n1 + " == " + n2);
        }
    }

    public static void assertEquals(Object o1, Object o2, String testName){
        if (!o1.equals(o2)){
            throw new RuntimeException("test \"" + testName + "\" FAILED: Assertion that " + o1 + " equals " + o2);
        }
    }

}