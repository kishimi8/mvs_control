clear
close
clc

mass = 1;

agents(1) = Agent(mass);
agents(2) = Agent(mass);
agents(3) = Agent(mass);

sys = System(agents);
sys.simulate(5);