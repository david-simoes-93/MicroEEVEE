package Engine;

import ciberIF.ciberIF;

public interface Behaviour {
    public String getName();
    public void execute(ciberIF cif);
    public boolean isPossible();
}
