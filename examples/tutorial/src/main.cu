#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <fstream>


#include "flamegpu/flame_api.h"
#include "flamegpu/runtime/flamegpu_api.h"
#include "flamegpu/io/factory.h"
#include "flamegpu/util/nvtx.h"

#define PRED_PREY_INTERACTION_RADIUS 0.1f
#define SAME_SPECIES_AVOIDANCE_RADIUS 0.1f
#define DELTA_TIME 0.1f
#define PRED_SPEED_ADVANTAGE 1.1f
#define GAIN_FROM_FOOD_PREDATOR 25
#define PRED_KILL_DISTANCE 0.05f



// Function definitions
void printPopulation(AgentPopulation &pop);

// FLAMEGPU_AGENT_FUNCTION(function_name, input_message_type, output_message_type)

// Predator functions

FLAMEGPU_AGENT_FUNCTION(pred_output_location, MsgNone, MsgBruteForce) {
    const float id = FLAMEGPU->getVariable<int>("id");
    const float x = FLAMEGPU->getVariable<float>("x");
    const float y = FLAMEGPU->getVariable<float>("y");
    FLAMEGPU->message_out.setVariable<int>("id", id); 
    FLAMEGPU->message_out.setVariable<float>("x", x); 
    FLAMEGPU->message_out.setVariable<float>("y", y); 

    return ALIVE;
}

FLAMEGPU_AGENT_FUNCTION(pred_follow_prey, MsgBruteForce, MsgNone) {
    // Fetch the predator's position
    const float predator_x = FLAMEGPU->getVariable<float>("x");
    const float predator_y = FLAMEGPU->getVariable<float>("y"); 

    // Find the closest prey by iterating the prey_location messages
    float closest_prey_x;
    float closest_prey_y;
    float closest_prey_distance = PRED_PREY_INTERACTION_RADIUS;
    int is_a_prey_in_range = 0;

    for (const auto& msg : FLAMEGPU->message_in) {
	// Fetch prey location
	const float prey_x = msg.getVariable<float>("x");
	const float prey_y = msg.getVariable<float>("y");

	// Check if prey is within sight range of predator
	const float dx = prey_x - predator_x;
	const float dy = prey_y - predator_y;
	const float distance = sqrt(dx*dx + dy*dy);

	// TODO: Move magic constant to env variable
	if (distance < 0.1f) {
	    closest_prey_x = prey_x;
	    closest_prey_y = prey_y;
	    closest_prey_distance = distance;
	    is_a_prey_in_range = 1;	
	} 
    }
    
    // If there was a prey in range, steer the predator towards it
    if (is_a_prey_in_range) {
	const float steer_x = closest_prey_x - predator_x;
	const float steer_y = closest_prey_y - predator_y;
	FLAMEGPU->setVariable<float>("steer_x", steer_x);	
	FLAMEGPU->setVariable<float>("steer_y", steer_y);	
    }

    return ALIVE;
}

FLAMEGPU_AGENT_FUNCTION(pred_avoid, MsgBruteForce, MsgNone) {
    // Fetch this predator's position
    const float predator_x = FLAMEGPU->getVariable<float>("x");
    const float predator_y = FLAMEGPU->getVariable<float>("y"); 
    float avoid_velocity_x = 0.0f;
    float avoid_velocity_y = 0.0f;

    // Add a steering factor away from each other predator. Strength increases with closeness.
    for (const auto& msg : FLAMEGPU->message_in) {
	// Fetch location of other predator
	const float other_predator_x = msg.getVariable<float>("x");
	const float other_predator_y = msg.getVariable<float>("y");

	// Check if the two predators are within interaction radius
	const float dx = predator_x - other_predator_x;
	const float dy = predator_y - other_predator_y;
	const float distance = sqrt(dx*dx + dy*dy);

	// TODO: Original implementation tests id to remove self-avoidance. However, dx, dy == 0 so no need?
	// TODO: Move magic constant to env variable
	if (distance < SAME_SPECIES_AVOIDANCE_RADIUS) {
	    avoid_velocity_x += (SAME_SPECIES_AVOIDANCE_RADIUS / distance) * dx;
	    avoid_velocity_y += (SAME_SPECIES_AVOIDANCE_RADIUS / distance) * dy;
	} 
    }
    
    FLAMEGPU->setVariable<float>("steer_x", avoid_velocity_x);
    FLAMEGPU->setVariable<float>("steer_y", avoid_velocity_y);

    return ALIVE;
}

FLAMEGPU_AGENT_FUNCTION(pred_move, MsgNone, MsgNone) {
    float predator_x = FLAMEGPU->getVariable<float>("x");
    float predator_y = FLAMEGPU->getVariable<float>("y");
    float predator_vx = FLAMEGPU->getVariable<float>("vx");
    float predator_vy = FLAMEGPU->getVariable<float>("vy");
    const float predator_fx = FLAMEGPU->getVariable<float>("steer_x");
    const float predator_fy = FLAMEGPU->getVariable<float>("steer_y");
    const float predator_life = FLAMEGPU->getVariable<int>("life");

    // TODO: There isn't any time scaling?
    // Integrate steering forces
    predator_vx += predator_fx;
    predator_vy += predator_fy;

    // TODO: Cap velocity

    // Integrate velocity
    predator_x += predator_vx * DELTA_TIME * PRED_SPEED_ADVANTAGE;
    predator_y += predator_vy * DELTA_TIME * PRED_SPEED_ADVANTAGE;

    // TODO: Bound the position within the environment

    // Update agent state
    FLAMEGPU->setVariable<float>("x", predator_x);
    FLAMEGPU->setVariable<float>("y", predator_y);
    FLAMEGPU->setVariable<float>("vx", predator_vx);
    FLAMEGPU->setVariable<float>("vy", predator_vy);

    // Reduce life by one unit of energy
    FLAMEGPU->setVariable<int>("life", predator_life -1);

    return ALIVE; 
}

FLAMEGPU_AGENT_FUNCTION(pred_eat_or_starve, MsgBruteForce, MsgNone) {
    const int predator_id = FLAMEGPU->getVariable<int>("id");
    int predator_life = FLAMEGPU->getVariable<int>("life");
    int isDead = 0;

    // Iterate prey_eaten messages to see if this predator ate a prey
    for (const auto& msg : FLAMEGPU->message_in) {
	if (msg.getVariable<int>("pred_id") == predator_id) {
	    predator_life += GAIN_FROM_FOOD_PREDATOR;
        }	    	
    }

    // Update agent state
    FLAMEGPU->setVariable<int>("life", predator_life);

    // Did the predator starve?
    if (predator_life < 1) {
        isDead = 1;
    }

    return isDead ? DEAD : ALIVE;
}

FLAMEGPU_AGENT_FUNCTION(pred_reproduction, MsgNone, MsgNone) {
    // TODO: NOT YET IMPLEMENTED
    return ALIVE;
}

// Prey functions

FLAMEGPU_AGENT_FUNCTION(prey_output_location, MsgNone, MsgBruteForce) {
    const float id = FLAMEGPU->getVariable<int>("id");
    const float x = FLAMEGPU->getVariable<float>("x");
    const float y = FLAMEGPU->getVariable<float>("y");
    FLAMEGPU->message_out.setVariable<int>("id", id); 
    FLAMEGPU->message_out.setVariable<float>("x", x); 
    FLAMEGPU->message_out.setVariable<float>("y", y); 

    return ALIVE;
}

FLAMEGPU_AGENT_FUNCTION(prey_avoid_pred, MsgBruteForce, MsgNone) {
    // Fetch this prey's position
    const float prey_x = FLAMEGPU->getVariable<float>("x");
    const float prey_y = FLAMEGPU->getVariable<float>("y"); 
    float avoid_velocity_x = 0.0f;
    float avoid_velocity_y = 0.0f;

    // Add a steering factor away from each predator. Strength increases with closeness.
    for (const auto& msg : FLAMEGPU->message_in) {
	// Fetch location of predator
	const float predator_x = msg.getVariable<float>("x");
	const float predator_y = msg.getVariable<float>("y");

	// Check if the two predators are within interaction radius
	const float dx = prey_x - predator_x;
	const float dy = prey_y - predator_y;
	const float distance = sqrt(dx*dx + dy*dy);

	if (distance < PRED_PREY_INTERACTION_RADIUS) {
	    // Steer the prey away from the predator
	    avoid_velocity_x += (PRED_PREY_INTERACTION_RADIUS / distance) * dx;
	    avoid_velocity_y += (PRED_PREY_INTERACTION_RADIUS / distance) * dy;
	} 
    }
   
    // Update agent state 
    FLAMEGPU->setVariable<float>("steer_x", avoid_velocity_x);
    FLAMEGPU->setVariable<float>("steer_y", avoid_velocity_y);

    return ALIVE;
}

FLAMEGPU_AGENT_FUNCTION(prey_flock, MsgBruteForce, MsgNone) {
    // TODO: NOT YET IMPLEMENTED

    return ALIVE;
}

FLAMEGPU_AGENT_FUNCTION(prey_move, MsgNone, MsgNone) {
    float prey_x = FLAMEGPU->getVariable<float>("x");
    float prey_y = FLAMEGPU->getVariable<float>("y");
    float prey_vx = FLAMEGPU->getVariable<float>("vx");
    float prey_vy = FLAMEGPU->getVariable<float>("vy");
    const float prey_fx = FLAMEGPU->getVariable<float>("steer_x");
    const float prey_fy = FLAMEGPU->getVariable<float>("steer_y");
    const float prey_life = FLAMEGPU->getVariable<int>("life");

    // TODO: There isn't any time scaling?
    // Integrate steering forces
    prey_vx += prey_fx;
    prey_vy += prey_fy;

    // TODO: Cap velocity

    // Integrate velocity
    prey_x += prey_vx * DELTA_TIME; 
    prey_y += prey_vy * DELTA_TIME;

    // TODO: Bound the position within the environment

    // Update agent state
    FLAMEGPU->setVariable<float>("x", prey_x);
    FLAMEGPU->setVariable<float>("y", prey_y);
    FLAMEGPU->setVariable<float>("vx", prey_vx);
    FLAMEGPU->setVariable<float>("vy", prey_vy);

    // Reduce life by one unit of energy
    FLAMEGPU->setVariable<int>("life", prey_life -1);

    return ALIVE; 
}

FLAMEGPU_AGENT_FUNCTION(prey_eaten, MsgBruteForce, MsgBruteForce) {
    int eaten = 0;
    int predator_id = -1;
    float closest_pred = PRED_KILL_DISTANCE;
    const float prey_x = FLAMEGPU->getVariable<float>("x");
    const float prey_y = FLAMEGPU->getVariable<float>("y");

    // Iterate predator_location messages to find the closest predator
    for (const auto& msg : FLAMEGPU->message_in) {
        // Fetch location of predator
	const float predator_x = msg.getVariable<float>("x");
	const float predator_y = msg.getVariable<float>("y");

	// Check if the two predators are within interaction radius
	const float dx = prey_x - predator_x;
	const float dy = prey_y - predator_y;
	const float distance = sqrt(dx*dx + dy*dy);

	if (distance < closest_pred) {
	    predator_id = msg.getVariable<int>("id");
	    closest_pred= distance;
            eaten = 1;
	}
    }

    if (eaten)
	// TODO: Output prey_eaten message
	int a = 0;
	

    return eaten ? DEAD : ALIVE;
}

FLAMEGPU_AGENT_FUNCTION(prey_eat_or_starve, MsgBruteForce, MsgNone) {
    int isDead = 0;

    // Exercise 3.3 : TODO: Describe exercise 

    return isDead ? DEAD : ALIVE;
}

FLAMEGPU_AGENT_FUNCTION(prey_reproduction, MsgNone, MsgNone) {
    // TODO: NOT YET IMPLEMENTED
    return ALIVE;
}

// Grass functions
FLAMEGPU_AGENT_FUNCTION(grass_output_location, MsgNone, MsgBruteForce) {
    // Exercise 3.1 : Set the variables for the grass_location message

    return ALIVE;
}

FLAMEGPU_AGENT_FUNCTION(grass_eaten, MsgBruteForce, MsgNone) {
    // Exercise 3.2 : TODO: Describe exercise

    return ALIVE;
}

FLAMEGPU_AGENT_FUNCTION(grass_growth, MsgNone, MsgNone) {
    // Exercise 3.4 : TODO: Describe exercise 

    return ALIVE;
}


// Model definition


int main(int argc, const char ** argv) {
    NVTX_RANGE("main");
    NVTX_PUSH("ModelDescription");
    ModelDescription model("Tutorial_PredatorPrey_Example");

    /**
     * MESSAGE DEFINITIONS
     */

    {   // Grass location message
        MsgBruteForce::Description &message = model.newMessage("grass_location_message");
        message.newVariable<int>("id");
	message.newVariable<float>("x");
	message.newVariable<float>("y");
    }
    {   // Predator location message
        MsgBruteForce::Description &message = model.newMessage("predator_location_message");
        message.newVariable<int>("id");
	message.newVariable<float>("x");
	message.newVariable<float>("y");
    }
    {   // Prey location message
        MsgBruteForce::Description &message = model.newMessage("prey_location_message");
        message.newVariable<int>("id");
	message.newVariable<float>("x");
	message.newVariable<float>("y");
    }
    {   // Grass eaten message
        MsgBruteForce::Description &message = model.newMessage("grass_eaten_message");
        message.newVariable<int>("id");
        message.newVariable<int>("prey_id");
    }
    {   // Prey eaten message
        MsgBruteForce::Description &message = model.newMessage("prey_eaten_message");
        message.newVariable<int>("id");
        message.newVariable<int>("pred_id");
    }


    /**
     * AGENT DEFINITIONS
     */

    {   // Prey agent
        AgentDescription &agent = model.newAgent("prey");
        agent.newVariable<int>("id");
	agent.newVariable<float>("x");
	agent.newVariable<float>("y");
	agent.newVariable<float>("vx");
	agent.newVariable<float>("vy");
	agent.newVariable<float>("steer_x");
	agent.newVariable<float>("steer_y");
        agent.newVariable<int>("life");
        agent.newVariable<float>("type")
;
        agent.newFunction("prey_output_location", prey_output_location).setMessageOutput("prey_location_message");
        agent.newFunction("prey_avoid_pred", prey_avoid_pred).setMessageInput("predator_location_message");
        agent.newFunction("prey_flock", prey_flock).setMessageInput("prey_location_message");
        agent.newFunction("prey_move", prey_move);
	// TODO: Add optional prey_eaten output message for prey_eaten
        auto& function = agent.newFunction("prey_eaten", prey_eaten);
	function.setMessageInput("predator_location_message");
	function.setMessageOutput("prey_eaten_message");
        agent.newFunction("prey_eat_or_starve", prey_eat_or_starve).setMessageInput("grass_eaten_message");
	// TODO: Output new agent??
        agent.newFunction("prey_reproduction", prey_reproduction);
    }

    {   // Predator agent
        AgentDescription &agent = model.newAgent("predator");
        agent.newVariable<int>("id");
        agent.newVariable<float>("x");
	agent.newVariable<float>("y");
	agent.newVariable<float>("vx");
	agent.newVariable<float>("vy");
	agent.newVariable<float>("steer_x");
	agent.newVariable<float>("steer_y");
        agent.newVariable<int>("life");
        agent.newVariable<float>("type");

        agent.newFunction("pred_output_location", pred_output_location).setMessageOutput("predator_location_message");
        agent.newFunction("pred_follow_prey", pred_follow_prey).setMessageInput("prey_location_message");
        agent.newFunction("pred_avoid", pred_avoid).setMessageInput("predator_location_message");
        agent.newFunction("pred_move", pred_move);
        agent.newFunction("pred_eat_or_starve", pred_eat_or_starve).setMessageInput("prey_eaten_message");
        agent.newFunction("pred_reproduction", pred_reproduction);
    }

    {   // Grass agent
	AgentDescription &agent = model.newAgent("grass");
        agent.newVariable<int>("id");
        agent.newVariable<float>("x");
	agent.newVariable<float>("y");
	agent.newVariable<int>("dead_cycles");
        agent.newVariable<int>("available");

	agent.newFunction("grass_output_location", grass_output_location).setMessageOutput("grass_location_message");
	// TODO: Add optional grass_eaten message
	agent.newFunction("grass_eaten", grass_eaten).setMessageInput("prey_location_message");
	agent.newFunction("grass_growth", grass_growth);
	
    }

   /**
     * ENVIRONMENT VARIABLES 
     */
    {
        EnvironmentDescription &env = model.Environment();
        env.add("REPRODUCE_PREY_PROB", 0.05f);
	env.add("REPRODUCE_PREDATOR_PROB", 0.03f);
	env.add("GAIN_FROM_FOOD_PREDATOR", 75);
	env.add("GAIN_FROM_FOOD_PREY", 50);
	env.add("GRASS_REGROW_CYCLES", 100);
    }

    /**
     * Control flow
     */
    {   // Layer #1
        LayerDescription &layer = model.newLayer();
        layer.addAgentFunction(prey_output_location);
        layer.addAgentFunction(pred_output_location);
        layer.addAgentFunction(grass_output_location);
    }
   // {   // Layer #2
   //     LayerDescription &layer = model.newLayer();
   //     layer.addAgentFunction(pred_follow_prey);
   //     layer.addAgentFunction(prey_avoid_pred);
   // }
   // {   // Layer #3
   //     LayerDescription &layer = model.newLayer();
   //     layer.addAgentFunction(prey_flock);
   //     layer.addAgentFunction(pred_avoid);
   // }
   // {   // Layer #4
   //     LayerDescription &layer = model.newLayer();
   //     layer.addAgentFunction(prey_move);
   //     layer.addAgentFunction(pred_move);
   // }
   // {   // Layer #5
   //     LayerDescription &layer = model.newLayer();
   //     layer.addAgentFunction(grass_eaten);
   //     layer.addAgentFunction(prey_eaten);
   // }
   // {   // Layer #6
   //     LayerDescription &layer = model.newLayer();
   //     layer.addAgentFunction(prey_eat_or_starve);
   //     layer.addAgentFunction(pred_eat_or_starve);
   // }
   // {   // Layer #7
   //     LayerDescription &layer = model.newLayer();
   //     layer.addAgentFunction(pred_reproduction);
   //     layer.addAgentFunction(prey_reproduction);
   //     layer.addAgentFunction(grass_growth);
   // }
    NVTX_POP();

    /**
     * Create Model Runner
     */
    NVTX_PUSH("CUDAAgentModel creation");
    CUDAAgentModel cuda_model(model);
    NVTX_POP();

    /**
     * Initialisation
     */
    cuda_model.initialise(argc, argv);
    if (cuda_model.getSimulationConfig().xml_input_file.empty()) {
	printf("XML Input was empty!");
    }


    /**
     * Execution
     */
    printf("Model initialised, beginning simulation...\n");
    printf("Step counter: %d\n Simulation Steps: %d", cuda_model.getStepCounter(), cuda_model.getSimulationConfig().steps);
     
    while (cuda_model.getStepCounter() < cuda_model.getSimulationConfig().steps && cuda_model.step()) {
	printf("On step %d", cuda_model.getStepCounter());
        //   cuda_model.getPopulationData(cell_pop);
        //  printPopulation(cell_pop);
        // getchar();
    }

   /**
    * Export Pop
    */
   // TODO
   return 0;
}

