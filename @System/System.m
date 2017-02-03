classdef System < handle
    
    %% PROPERTIES
    properties
        agents;
    end
    
    %% METHODS
    methods
        %% COSTRUCTOR
        function obj = System(agents)
            obj.agents = agents;
        end
        simulate(obj,duration);
        
    end
end
