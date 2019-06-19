# -------------------------------------------------------
# Add your custom functions at the bottom of the file
# and the server counterpart to lua/b0RemoteApiServer.lua
# -------------------------------------------------------

# import b0
from gym_vrep.envs import b0
import msgpack
import random
import string
import time

class RemoteApiClient:
    def __init__(self,nodeName='b0RemoteApi_pythonClient',channelName='b0RemoteApi',inactivityToleranceInSec=60,setupSubscribersAsynchronously=False):
        self._channelName=channelName
        self._serviceCallTopic=channelName+'SerX'
        self._defaultPublisherTopic=channelName+'SubX'
        self._defaultSubscriberTopic=channelName+'PubX'
        self._nextDefaultSubscriberHandle=2
        self._nextDedicatedPublisherHandle=500
        self._nextDedicatedSubscriberHandle=1000
        b0.init()
        self._node=b0.Node(nodeName)
        self._clientId=''.join(random.choice(string.ascii_uppercase+string.ascii_lowercase+string.digits) for _ in range(10))
        self._serviceClient=b0.ServiceClient(self._node,self._serviceCallTopic)
        self._serviceClient.set_option(3,250000) #read timeout of 1000ms
        self._defaultPublisher=b0.Publisher(self._node,self._defaultPublisherTopic)
        self._defaultSubscriber=b0.Subscriber(self._node,self._defaultSubscriberTopic,None) # we will poll the socket
        print('\n  Running B0 Remote API client with channel name ['+channelName+']')
        print('  make sure that: 1) the B0 resolver is running')
        print('                  2) V-REP is running the B0 Remote API server with the same channel name')
        print('  Initializing...\n')
        self._node.init()
        self._handleFunction('inactivityTolerance',[inactivityToleranceInSec],self._serviceCallTopic)
        print('\n  Connected!\n')
        self._allSubscribers={}
        self._allDedicatedPublishers={}
        self._setupSubscribersAsynchronously=setupSubscribersAsynchronously
  
    def __enter__(self):
        return self
    
    def __exit__(self,*err):
        self._pongReceived=False
        self._handleFunction('Ping',[0],self.simxDefaultSubscriber(self._pingCallback))
        while not self._pongReceived:
            self.simxSpinOnce();
        self._handleFunction('DisconnectClient',[self._clientId],self._serviceCallTopic)
        for key, value in self._allSubscribers.items():
            if value['handle']!=self._defaultSubscriber:
                value['handle'].cleanup()
        for key, value in self._allDedicatedPublishers.items():
            value.cleanup()
        self._node.cleanup()
        
    def _pingCallback(self,msg):
        self._pongReceived=True
        
    def _handleReceivedMessage(self,msg):
        msg=msgpack.unpackb(msg)
        msg[0]=msg[0].decode('ascii')
        if msg[0] in self._allSubscribers:
            cbMsg=msg[1]
            if len(cbMsg)==1:
                cbMsg.append(None)
            self._allSubscribers[msg[0]]['cb'](cbMsg)
            
    def _handleFunction(self,funcName,reqArgs,topic):
        if topic==self._serviceCallTopic:
            packedData=msgpack.packb([[funcName,self._clientId,topic,0],reqArgs])
            rep = msgpack.unpackb(self._serviceClient.call(packedData))
            if len(rep)==1:
                rep.append(None)
            return rep
        elif topic==self._defaultPublisherTopic:
            packedData=msgpack.packb([[funcName,self._clientId,topic,1],reqArgs])
            self._defaultPublisher.publish(packedData)
        elif topic in self._allSubscribers:
            if self._allSubscribers[topic]['handle']==self._defaultSubscriber:
                packedData=msgpack.packb([[funcName,self._clientId,topic,2],reqArgs])
                if self._setupSubscribersAsynchronously:
                    self._defaultPublisher.publish(packedData)
                else:
                    self._serviceClient.call(packedData)
            else:
                packedData=msgpack.packb([[funcName,self._clientId,topic,4],reqArgs])
                if self._setupSubscribersAsynchronously:
                    self._defaultPublisher.publish(packedData)
                else:
                    self._serviceClient.call(packedData)
        elif topic in self._allDedicatedPublishers:
            packedData=msgpack.packb([[funcName,self._clientId,topic,3],reqArgs])
            self._allDedicatedPublishers[topic].publish(packedData)
        else:
            print('B0 Remote API error: invalid topic')
        
    def simxDefaultPublisher(self):
        return self._defaultPublisherTopic

    def simxCreatePublisher(self,dropMessages=False):
        topic=self._channelName+'Sub'+str(self._nextDedicatedPublisherHandle)+self._clientId
        self._nextDedicatedPublisherHandle=self._nextDedicatedPublisherHandle+1
        pub=b0.Publisher(self._node,topic,0,1)
        pub.init()
        self._allDedicatedPublishers[topic]=pub
        self._handleFunction('createSubscriber',[topic,dropMessages],self._serviceCallTopic)
        return topic

    def simxDefaultSubscriber(self,cb,publishInterval=1):
        topic=self._channelName+'Pub'+str(self._nextDefaultSubscriberHandle)+self._clientId
        self._nextDefaultSubscriberHandle=self._nextDefaultSubscriberHandle+1
        self._allSubscribers[topic]={}
        self._allSubscribers[topic]['handle']=self._defaultSubscriber
        self._allSubscribers[topic]['cb']=cb
        self._allSubscribers[topic]['dropMessages']=False
        channel=self._serviceCallTopic
        if self._setupSubscribersAsynchronously:
            channel=self._defaultPublisherTopic
        self._handleFunction('setDefaultPublisherPubInterval',[topic,publishInterval],channel)
        return topic
        
    def simxCreateSubscriber(self,cb,publishInterval=1,dropMessages=False):
        topic=self._channelName+'Pub'+str(self._nextDedicatedSubscriberHandle)+self._clientId
        self._nextDedicatedSubscriberHandle=self._nextDedicatedSubscriberHandle+1
        sub=b0.Subscriber(self._node,topic,None,0,1)
        if dropMessages:
            sub.set_option(6,1) #conflate option enabled
        else:
            sub.set_option(6,0) #conflate option disabled
        sub.init()
        self._allSubscribers[topic]={}
        self._allSubscribers[topic]['handle']=sub
        self._allSubscribers[topic]['cb']=cb
        self._allSubscribers[topic]['dropMessages']=dropMessages
        channel=self._serviceCallTopic
        if self._setupSubscribersAsynchronously:
            channel=self._defaultPublisherTopic
        self._handleFunction('createPublisher',[topic,publishInterval],channel)
        return topic
  
    def simxServiceCall(self):
        return self._serviceCallTopic
        
    def simxSpin(self):
        while True:
            self.simxSpinOnce()
        
    def simxSpinOnce(self):
        defaultSubscriberAlreadyProcessed=False
        for key, value in self._allSubscribers.items():
            readData=None
            if (value['handle']!=self._defaultSubscriber) or (not defaultSubscriberAlreadyProcessed):
                defaultSubscriberAlreadyProcessed=defaultSubscriberAlreadyProcessed or (value['handle']==self._defaultSubscriber)
                while value['handle'].poll(0):
                    readData=value['handle'].read()
                    if not value['dropMessages']:
                        self._handleReceivedMessage(readData)
                if value['dropMessages'] and (readData is not None):
                    self._handleReceivedMessage(readData)
                    
    def simxGetTimeInMs(self):
        return self._node.hardware_time_usec()/1000;    

    def simxSleep(self,durationInMs):
        time.sleep(durationInMs)
        
    def simxSynchronous(self,enable):
        reqArgs = [enable]
        funcName = 'Synchronous'
        self._handleFunction(funcName,reqArgs,self._serviceCallTopic)
        
    def simxSynchronousTrigger(self):
        reqArgs = [0]
        funcName = 'SynchronousTrigger'
        self._handleFunction(funcName,reqArgs,self._defaultPublisherTopic)
        
    def simxGetSimulationStepDone(self,topic):
        if topic in self._allSubscribers:
            reqArgs = [0]
            funcName = 'GetSimulationStepDone'
            self._handleFunction(funcName,reqArgs,topic)
        else:
            print('B0 Remote API error: invalid topic')
        
    def simxGetSimulationStepStarted(self,topic):
        if topic in self._allSubscribers:
            reqArgs = [0]
            funcName = 'GetSimulationStepStarted'
            self._handleFunction(funcName,reqArgs,topic)
        else:
            print('B0 Remote API error: invalid topic')
    
    def simxCallScriptFunction(self,funcAtObjName,scriptType,arg,topic):
        packedArg=msgpack.packb(arg)
        reqArgs = [funcAtObjName,scriptType,packedArg]
        funcName = 'CallScriptFunction'
        return self._handleFunction(funcName,reqArgs,topic)
        
    def simxGetObjectHandle(self,objectName,topic):
        reqArgs = [objectName]
        return self._handleFunction('GetObjectHandle',reqArgs,topic)
    def simxAddStatusbarMessage(self,msg,topic):
        reqArgs = [msg]
        return self._handleFunction('AddStatusbarMessage',reqArgs,topic)
    def simxGetObjectPosition(self,objectHandle,relObjHandle,topic):
        reqArgs = [objectHandle,relObjHandle]
        return self._handleFunction('GetObjectPosition',reqArgs,topic)
    def simxGetObjectOrientation(self,objectHandle,relObjHandle,topic):
        reqArgs = [objectHandle,relObjHandle]
        return self._handleFunction('GetObjectOrientation',reqArgs,topic)
    def simxGetObjectQuaternion(self,objectHandle,relObjHandle,topic):
        reqArgs = [objectHandle,relObjHandle]
        return self._handleFunction('GetObjectQuaternion',reqArgs,topic)
    def simxGetObjectPose(self,objectHandle,relObjHandle,topic):
        reqArgs = [objectHandle,relObjHandle]
        return self._handleFunction('GetObjectPose',reqArgs,topic)
    def simxGetObjectMatrix(self,objectHandle,relObjHandle,topic):
        reqArgs = [objectHandle,relObjHandle]
        return self._handleFunction('GetObjectMatrix',reqArgs,topic)
    def simxSetObjectPosition(self,objectHandle,relObjHandle,position,topic):
        reqArgs = [objectHandle,relObjHandle,position]
        return self._handleFunction('SetObjectPosition',reqArgs,topic)
    def simxSetObjectOrientation(self,objectHandle,relObjHandle,euler,topic):
        reqArgs = [objectHandle,relObjHandle,euler]
        return self._handleFunction('SetObjectOrientation',reqArgs,topic)
    def simxSetObjectQuaternion(self,objectHandle,relObjHandle,quat,topic):
        reqArgs = [objectHandle,relObjHandle,quat]
        return self._handleFunction('SetObjectQuaternion',reqArgs,topic)
    def simxSetObjectPose(self,objectHandle,relObjHandle,pose,topic):
        reqArgs = [objectHandle,relObjHandle,pose]
        return self._handleFunction('SetObjectPose',reqArgs,topic)
    def simxSetObjectMatrix(self,objectHandle,relObjHandle,matr,topic):
        reqArgs = [objectHandle,relObjHandle,matr]
        return self._handleFunction('SetObjectMatrix',reqArgs,topic)
    def simxClearFloatSignal(self,sigName,topic):
        reqArgs = [sigName]
        return self._handleFunction('ClearFloatSignal',reqArgs,topic)
    def simxClearIntegerSignal(self,sigName,topic):
        reqArgs = [sigName]
        return self._handleFunction('ClearIntegerSignal',reqArgs,topic)
    def simxClearStringSignal(self,sigName,topic):
        reqArgs = [sigName]
        return self._handleFunction('ClearStringSignal',reqArgs,topic)
    def simxSetFloatSignal(self,sigName,sigValue,topic):
        reqArgs = [sigName,sigValue]
        return self._handleFunction('SetFloatSignal',reqArgs,topic)
    def simxSetIntSignal(self,sigName,sigValue,topic):
        reqArgs = [sigName,sigValue]
        return self._handleFunction('SetIntSignal',reqArgs,topic)
    def simxSetStringSignal(self,sigName,sigValue,topic):
        reqArgs = [sigName,sigValue]
        return self._handleFunction('SetStringSignal',reqArgs,topic)
    def simxGetFloatSignal(self,sigName,topic):
        reqArgs = [sigName]
        return self._handleFunction('GetFloatSignal',reqArgs,topic)
    def simxGetIntSignal(self,sigName,topic):
        reqArgs = [sigName]
        return self._handleFunction('GetIntSignal',reqArgs,topic)
    def simxGetStringSignal(self,sigName,topic):
        reqArgs = [sigName]
        return self._handleFunction('GetStringSignal',reqArgs,topic)
    def simxAuxiliaryConsoleClose(self,consoleHandle,topic):
        reqArgs = [consoleHandle]
        return self._handleFunction('AuxiliaryConsoleClose',reqArgs,topic)
    def simxAuxiliaryConsolePrint(self,consoleHandle,text,topic):
        reqArgs = [consoleHandle,text]
        return self._handleFunction('AuxiliaryConsolePrint',reqArgs,topic)
    def simxAuxiliaryConsoleShow(self,consoleHandle,showState,topic):
        reqArgs = [consoleHandle,showState]
        return self._handleFunction('AuxiliaryConsoleShow',reqArgs,topic)
    def simxAuxiliaryConsoleOpen(self,title,maxLines,mode,position,size,textColor,backgroundColor,topic):
        reqArgs = [title,maxLines,mode,position,size,textColor,backgroundColor]
        return self._handleFunction('AuxiliaryConsoleOpen',reqArgs,topic)
    def simxStartSimulation(self,topic):
        reqArgs = [0]
        return self._handleFunction('StartSimulation',reqArgs,topic)
    def simxStopSimulation(self,topic):
        reqArgs = [0]
        return self._handleFunction('StopSimulation',reqArgs,topic)
    def simxPauseSimulation(self,topic):
        reqArgs = [0]
        return self._handleFunction('PauseSimulation',reqArgs,topic)
    def simxGetVisionSensorImage(self,objectHandle,greyScale,topic):
        reqArgs = [objectHandle,greyScale]
        return self._handleFunction('GetVisionSensorImage',reqArgs,topic)
    def simxSetVisionSensorImage(self,objectHandle,greyScale,img,topic):
        reqArgs = [objectHandle,greyScale,img]
        return self._handleFunction('SetVisionSensorImage',reqArgs,topic)
    def simxGetVisionSensorDepthBuffer(self,objectHandle,toMeters,asByteArray,topic):
        reqArgs = [objectHandle,toMeters,asByteArray]
        return self._handleFunction('GetVisionSensorDepthBuffer',reqArgs,topic)
    def simxAddDrawingObject_points(self,size,color,coords,topic):
        reqArgs = [size,color,coords]
        return self._handleFunction('AddDrawingObject_points',reqArgs,topic)
    def simxAddDrawingObject_spheres(self,size,color,coords,topic):
        reqArgs = [size,color,coords]
        return self._handleFunction('AddDrawingObject_spheres',reqArgs,topic)
    def simxAddDrawingObject_cubes(self,size,color,coords,topic):
        reqArgs = [size,color,coords]
        return self._handleFunction('AddDrawingObject_cubes',reqArgs,topic)
    def simxAddDrawingObject_segments(self,lineSize,color,segments,topic):
        reqArgs = [lineSize,color,segments]
        return self._handleFunction('AddDrawingObject_segments',reqArgs,topic)
    def simxAddDrawingObject_triangles(self,color,triangles,topic):
        reqArgs = [color,triangles]
        return self._handleFunction('AddDrawingObject_triangles',reqArgs,topic)
    def simxRemoveDrawingObject(self,handle,topic):
        reqArgs = [handle]
        return self._handleFunction('RemoveDrawingObject',reqArgs,topic)
    def simxGetCollisionHandle(self,nameOfObject,topic):
        reqArgs = [nameOfObject]
        return self._handleFunction('GetCollisionHandle',reqArgs,topic)
    def simxGetDistanceHandle(self,nameOfObject,topic):
        reqArgs = [nameOfObject]
        return self._handleFunction('GetDistanceHandle',reqArgs,topic)
    def simxReadCollision(self,handle,topic):
        reqArgs = [handle]
        return self._handleFunction('ReadCollision',reqArgs,topic)
    def simxReadDistance(self,handle,topic):
        reqArgs = [handle]
        return self._handleFunction('ReadDistance',reqArgs,topic)
    def simxCheckCollision(self,entity1,entity2,topic):
        reqArgs = [entity1,entity2]
        return self._handleFunction('CheckCollision',reqArgs,topic)
    def simxCheckDistance(self,entity1,entity2,threshold,topic):
        reqArgs = [entity1,entity2,threshold]
        return self._handleFunction('CheckDistance',reqArgs,topic)
    def simxReadProximitySensor(self,handle,topic):
        reqArgs = [handle]
        return self._handleFunction('ReadProximitySensor',reqArgs,topic)
    def simxCheckProximitySensor(self,handle,entity,topic):
        reqArgs = [handle,entity]
        return self._handleFunction('CheckProximitySensor',reqArgs,topic)
    def simxReadForceSensor(self,handle,topic):
        reqArgs = [handle]
        return self._handleFunction('ReadForceSensor',reqArgs,topic)
    def simxBreakForceSensor(self,handle,topic):
        reqArgs = [handle]
        return self._handleFunction('BreakForceSensor',reqArgs,topic)
    def simxReadVisionSensor(self,handle,topic):
        reqArgs = [handle]
        return self._handleFunction('ReadVisionSensor',reqArgs,topic)
    def simxCheckVisionSensor(self,handle,entity,topic):
        reqArgs = [handle,entity]
        return self._handleFunction('CheckVisionSensor',reqArgs,topic)
    def simxCopyPasteObjects(self,objectHandles,options,topic):
        reqArgs = [objectHandles,options]
        return self._handleFunction('CopyPasteObjects',reqArgs,topic)
    def simxRemoveObjects(self,objectHandles,options,topic):
        reqArgs = [objectHandles,options]
        return self._handleFunction('RemoveObjects',reqArgs,topic)
    def simxCloseScene(self,topic):
        reqArgs = [0]
        return self._handleFunction('CloseScene',reqArgs,topic)
    def simxSetStringParameter(self,paramId,paramVal,topic):
        reqArgs = [paramId,paramVal]
        return self._handleFunction('SetStringParameter',reqArgs,topic)
    def simxSetFloatParameter(self,paramId,paramVal,topic):
        reqArgs = [paramId,paramVal]
        return self._handleFunction('SetFloatParameter',reqArgs,topic)
    def simxSetArrayParameter(self,paramId,paramVal,topic):
        reqArgs = [paramId,paramVal]
        return self._handleFunction('SetArrayParameter',reqArgs,topic)
    def simxSetIntParameter(self,paramId,paramVal,topic):
        reqArgs = [paramId,paramVal]
        return self._handleFunction('SetIntParameter',reqArgs,topic)
    def simxSetBoolParameter(self,paramId,paramVal,topic):
        reqArgs = [paramId,paramVal]
        return self._handleFunction('SetBoolParameter',reqArgs,topic)
    def simxGetStringParameter(self,paramId,topic):
        reqArgs = [paramId]
        return self._handleFunction('GetStringParameter',reqArgs,topic)
    def simxGetFloatParameter(self,paramId,topic):
        reqArgs = [paramId]
        return self._handleFunction('GetFloatParameter',reqArgs,topic)
    def simxGetArrayParameter(self,paramId,topic):
        reqArgs = [paramId]
        return self._handleFunction('GetArrayParameter',reqArgs,topic)
    def simxGetIntParameter(self,paramId,topic):
        reqArgs = [paramId]
        return self._handleFunction('GetIntParameter',reqArgs,topic)
    def simxGetBoolParameter(self,paramId,topic):
        reqArgs = [paramId]
        return self._handleFunction('GetBoolParameter',reqArgs,topic)
    def simxDisplayDialog(self,titleText,mainText,dialogType,inputText,topic):
        reqArgs = [titleText,mainText,dialogType,inputText]
        return self._handleFunction('DisplayDialog',reqArgs,topic)
    def simxGetDialogResult(self,handle,topic):
        reqArgs = [handle]
        return self._handleFunction('GetDialogResult',reqArgs,topic)
    def simxGetDialogInput(self,handle,topic):
        reqArgs = [handle]
        return self._handleFunction('GetDialogInput',reqArgs,topic)
    def simxEndDialog(self,handle,topic):
        reqArgs = [handle]
        return self._handleFunction('EndDialog',reqArgs,topic)
    def simxExecuteScriptString(self,code,topic):
        reqArgs = [code]
        return self._handleFunction('ExecuteScriptString',reqArgs,topic)
    def simxGetCollectionHandle(self,collectionName,topic):
        reqArgs = [collectionName]
        return self._handleFunction('GetCollectionHandle',reqArgs,topic)
    def simxGetJointForce(self,jointHandle,topic):
        reqArgs = [jointHandle]
        return self._handleFunction('GetJointForce',reqArgs,topic)
    def simxSetJointForce(self,jointHandle,forceOrTorque,topic):
        reqArgs = [jointHandle,forceOrTorque]
        return self._handleFunction('SetJointForce',reqArgs,topic)
    def simxGetJointPosition(self,jointHandle,topic):
        reqArgs = [jointHandle]
        return self._handleFunction('GetJointPosition',reqArgs,topic)
    def simxSetJointPosition(self,jointHandle,position,topic):
        reqArgs = [jointHandle,position]
        return self._handleFunction('SetJointPosition',reqArgs,topic)
    def simxGetJointTargetPosition(self,jointHandle,topic):
        reqArgs = [jointHandle]
        return self._handleFunction('GetJointTargetPosition',reqArgs,topic)
    def simxSetJointTargetPosition(self,jointHandle,targetPos,topic):
        reqArgs = [jointHandle,targetPos]
        return self._handleFunction('SetJointTargetPosition',reqArgs,topic)
    def simxGetJointTargetVelocity(self,jointHandle,topic):
        reqArgs = [jointHandle]
        return self._handleFunction('GetJointTargetVelocity',reqArgs,topic)
    def simxSetJointTargetVelocity(self,jointHandle,targetPos,topic):
        reqArgs = [jointHandle,targetPos]
        return self._handleFunction('SetJointTargetVelocity',reqArgs,topic)
    def simxGetObjectChild(self,objectHandle,index,topic):
        reqArgs = [objectHandle,index]
        return self._handleFunction('GetObjectChild',reqArgs,topic)
    def simxGetObjectParent(self,objectHandle,topic):
        reqArgs = [objectHandle]
        return self._handleFunction('GetObjectParent',reqArgs,topic)
    def simxSetObjectParent(self,objectHandle,parentHandle,assembly,keepInPlace,topic):
        reqArgs = [objectHandle,parentHandle,assembly,keepInPlace]
        return self._handleFunction('SetObjectParent',reqArgs,topic)
    def simxGetObjectsInTree(self,treeBaseHandle,objectType,options,topic):
        reqArgs = [treeBaseHandle,objectType,options]
        return self._handleFunction('GetObjectsInTree',reqArgs,topic)
    def simxGetObjectName(self,objectHandle,altName,topic):
        reqArgs = [objectHandle,altName]
        return self._handleFunction('GetObjectName',reqArgs,topic)
    def simxGetObjectFloatParameter(self,objectHandle,parameterID,topic):
        reqArgs = [objectHandle,parameterID]
        return self._handleFunction('GetObjectFloatParameter',reqArgs,topic)
    def simxGetObjectIntParameter(self,objectHandle,parameterID,topic):
        reqArgs = [objectHandle,parameterID]
        return self._handleFunction('GetObjectIntParameter',reqArgs,topic)
    def simxGetObjectStringParameter(self,objectHandle,parameterID,topic):
        reqArgs = [objectHandle,parameterID]
        return self._handleFunction('GetObjectStringParameter',reqArgs,topic)
    def simxSetObjectFloatParameter(self,objectHandle,parameterID,parameter,topic):
        reqArgs = [objectHandle,parameterID,parameter]
        return self._handleFunction('SetObjectFloatParameter',reqArgs,topic)
    def simxSetObjectIntParameter(self,objectHandle,parameterID,parameter,topic):
        reqArgs = [objectHandle,parameterID,parameter]
        return self._handleFunction('SetObjectIntParameter',reqArgs,topic)
    def simxSetObjectStringParameter(self,objectHandle,parameterID,parameter,topic):
        reqArgs = [objectHandle,parameterID,parameter]
        return self._handleFunction('SetObjectStringParameter',reqArgs,topic)
    def simxGetSimulationTime(self,topic):
        reqArgs = [0]
        return self._handleFunction('GetSimulationTime',reqArgs,topic)
    def simxGetSimulationTimeStep(self,topic):
        reqArgs = [0]
        return self._handleFunction('GetSimulationTimeStep',reqArgs,topic)
    def simxGetServerTimeInMs(self,topic):
        reqArgs = [0]
        return self._handleFunction('GetServerTimeInMs',reqArgs,topic)
    def simxGetSimulationState(self,topic):
        reqArgs = [0]
        return self._handleFunction('GetSimulationState',reqArgs,topic)
    def simxEvaluateToInt(self,str,topic):
        reqArgs = [str]
        return self._handleFunction('EvaluateToInt',reqArgs,topic)
    def simxEvaluateToStr(self,str,topic):
        reqArgs = [str]
        return self._handleFunction('EvaluateToStr',reqArgs,topic)
    def simxGetObjects(self,objectType,topic):
        reqArgs = [objectType]
        return self._handleFunction('GetObjects',reqArgs,topic)
    def simxCreateDummy(self,size,color,topic):
        reqArgs = [size,color]
        return self._handleFunction('CreateDummy',reqArgs,topic)
    def simxGetObjectSelection(self,topic):
        reqArgs = [0]
        return self._handleFunction('GetObjectSelection',reqArgs,topic)
    def simxSetObjectSelection(self,selection,topic):
        reqArgs = [selection]
        return self._handleFunction('SetObjectSelection',reqArgs,topic)
    def simxGetObjectVelocity(self,handle,topic):
        reqArgs = [handle]
        return self._handleFunction('GetObjectVelocity',reqArgs,topic)
    def simxLoadModelFromFile(self,filename,topic):
        reqArgs = [filename]
        return self._handleFunction('LoadModelFromFile',reqArgs,topic)
    def simxLoadModelFromBuffer(self,buffer,topic):
        reqArgs = [buffer]
        return self._handleFunction('LoadModelFromBuffer',reqArgs,topic)
    def simxLoadScene(self,filename,topic):
        reqArgs = [filename]
        return self._handleFunction('LoadScene',reqArgs,topic)

    # -----------------------------------------------------------
    # Add your custom functions here, or even better,
    # add them to b0RemoteApiBindings/generate/simxFunctions.xml,
    # and generate this file again.
    # Then add the server part of your custom functions at the
    # beginning of file lua/b0RemoteApiServer.lua
    # -----------------------------------------------------------
