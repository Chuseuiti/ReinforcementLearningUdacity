import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.behavior.statehashing.DiscreteStateHashFactory;
import burlap.domain.singleagent.graphdefined.GraphDefinedDomain;
import burlap.oomdp.auxiliary.DomainGenerator;
import burlap.oomdp.auxiliary.common.NullTermination;
import burlap.oomdp.core.*;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;

public class MDPSolver {
    DomainGenerator dg;
    Domain domain;
    
    RewardFunction rf; 
    TerminalFunction tf;
    
    DiscreteStateHashFactory hashFactory;
    // Initial State
    int initState_number = 0;
    State initState;
    
    
    public MDPSolver(int numStates, int numActions, double[][][] probabilitiesOfTransitions, 
                     double[][][] rewards) {

        //Define Graph
        this.dg = new GraphDefinedDomain(numStates);
                         
        // Define transitions
        for(int state = 0; state<probabilitiesOfTransitions.length; state++){
            for(int action = 0; action<probabilitiesOfTransitions[0].length; action++){
                for(int state_final = 0; state_final<probabilitiesOfTransitions[0][0].length; state_final++){
                    ((GraphDefinedDomain)this.dg).setTransition(state, action, state_final, probabilitiesOfTransitions[state][action][state_final]);
                }
            }        
        } 

        //Define rewards
        this.rf = new DesignerRewards(rewards, probabilitiesOfTransitions);   
        
        //Define Domain
        this.domain = this.dg.generateDomain();
        
        //Define initialization state 
        this.initState = GraphDefinedDomain.getState(this.domain, initState_number);
        
        //Define terinal state 
        this.tf = new NullTermination();
        
        //Hash factory
        this.hashFactory = new DiscreteStateHashFactory();

    }
    public Domain getDomain(){
        return this.domain;
    }
    public static class DesignerRewards implements RewardFunction{
        double[][][] rewards_designer;
        double[][][] probabilitiesOfTransitions_temp;

        public DesignerRewards(double[][][] rewards, double[][][] probabilitiesOfTransitions){
            this.rewards_designer = rewards;
            this.probabilitiesOfTransitions_temp = probabilitiesOfTransitions;
            
        }
        
        // Override reward method
        @Override
        public double reward(State state, GroundedAction action, State state_final){
            //Incorrect conversion
            
            int int_state = GraphDefinedDomain.getNodeId(state);
            //int int_action = (int)action;//GraphDefinedDomain.getActionId(action);
            int int_state_final = GraphDefinedDomain.getNodeId(state_final);
            //Here it should go the conversion from GroundedAction to int, but I couldnt find the correct method to do the conversion.
            double value_higher_reward = 0;
            for(int ii = 0; ii < (this.probabilitiesOfTransitions_temp)[int_state].length; ii++){
                
                double rewards_action_temp = this.rewards_designer[int_state][ii][int_state_final];
                
                if(value_higher_reward <= rewards_action_temp) {
                    value_higher_reward = rewards_action_temp;
                }
                
            }
            return value_higher_reward;
            
        }
    }

    public double valueOfState(int state, double gamma) {
    
        //this.initState = this.domain.getState(this.domain, state);
        ValueIteration vi = computeValue(gamma);
        State s = GraphDefinedDomain.getState(this.domain,state);
        return vi.value(s); 
        
    }
    
    //Value Iteration - Iterative Bellman equation
    private ValueIteration computeValue(double gamma){
        double maxDelta = 0.0001;
        int maxIterations = 1000;

        ValueIteration vi = new ValueIteration(this.domain, rf, this.tf, gamma, this.hashFactory, maxDelta, maxIterations);
        vi.planFromState(this.initState);
        return vi;
    }
    
}
