package Engine;

public class Repetitions {

    static int[] history = new int[8];
    static int counter = 0, tick=0;
    static final int RIGHT=0, LEFT=1;

    public static boolean avoidRepetitions() {
        if(tick>0)
        {
            tick--;
            return true;
        }
        
        int[] vals = new int[2];
        int prev = -1;

        for (int i = 1; i < history.length; i++) {
            if (i > 0) {
                prev = history[i - 1];
            } else {
                prev = history[history.length - 1];
            }

            for (int j = 0; j < 4; j++) {
                if (history[i] == j && history[i] != prev) {
                    vals[j]++;
                    break;
                }
            }
        }

        int counterL = 0;
        for (int i = 0; i < vals.length; i++) {
            if (vals[i] != 0) {
                counterL++;
            }
        }

        if (counterL == 2) {
            boolean repetition = vals[0]==history.length / 2;
            tick=3;
            System.out.print("\tLeft turns="+vals[LEFT] + " and Right turns="+vals[RIGHT]);
            
            return repetition;
        }

        return false;
    }
    
    public static int getMainSideTurning(double modder){
        return (int)(0.5 + modder*0.5);
    }
    
    public static int getWeakSideTurning(double modder){
        return (int)(0.5 - modder*0.5);
    }
}
