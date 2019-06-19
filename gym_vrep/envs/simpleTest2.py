import b0RemoteApi
import time

with b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient','b0RemoteApi') as client:    
    doNextStep=True

    def simulationStepStarted(msg):
        simTime=msg[1][b'simulationTime'];
        print('Simulation step started. Simulation time: ',simTime)
        
    def simulationStepDone(msg):
        simTime=msg[1][b'simulationTime'];
        print('Simulation step done. Simulation time: ',simTime);
        global doNextStep
        doNextStep=True
        
    client.simxSynchronous(True)
    client.simxGetSimulationStepStarted(client.simxDefaultSubscriber(simulationStepStarted));
    client.simxGetSimulationStepDone(client.simxDefaultSubscriber(simulationStepDone));
    client.simxStartSimulation(client.simxDefaultPublisher())
    
    startTime=time.time()
    while time.time()<startTime+5: 
        if doNextStep:
            doNextStep=False
            client.simxSynchronousTrigger()
        client.simxSpinOnce()
    client.simxStopSimulation(client.simxDefaultPublisher())

