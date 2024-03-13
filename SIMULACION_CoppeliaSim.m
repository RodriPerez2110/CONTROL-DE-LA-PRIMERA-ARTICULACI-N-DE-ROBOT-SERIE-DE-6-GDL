% Level-2 MATLAB S-Function:

function SIMULACION_CoppeliaSim(block)
    %MSFUNTMPL_BASIC A Template for a Level-2 MATLAB S-Function
    %   The MATLAB S-function is written as a MATLAB function with the

    %   same name as the S-function. Replace 'msfuntmpl_basic' with the

    %   name of your S-function.

    %   Copyright 2003-2018 The MathWorks, Inc.

    %%
    %% The setup method is used to set up the basic attributes of the
     %% S-function such as ports, parameters, etc. Do not add any other
    %% calls to the main body of the function.
    %%
    setup(block);
end %function

%% Function: setup ===================================================
%% Abstract:
%%   Set up the basic characteristics of the S-function block such as:
%%   - Input ports
%%   - Output ports
%%   - Dialog parameters
%%   - Options
%%
%%   Required         : Yes
%%   C MEX counterpart: mdlInitializeSizes
function setup(block)
    % Register number of ports:
    block.NumInputPorts  = 1;
    block.NumOutputPorts = 2;

    % Setup port properties to be inherited or dynamic:
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;

    % Override input port properties:
    block.InputPort(1).Dimensions  = [6 1];  %[6x1]
    block.InputPort(1).DatatypeID  = 0;      % double
    block.InputPort(1).Complexity  = 'Real';
    block.InputPort(1).SamplingMode = 'Sample';
    block.InputPort(1).DirectFeedthrough = true;
    block.InputPort(1).DirectFeedthrough = true;

    % Override output ports properties:
    block.OutputPort(1).Dimensions  = 1; %Valor del Bloque STOP de Simulink.
    block.OutputPort(1).DatatypeID  = 8;      % boolean
    block.OutputPort(1).Complexity  = 'Real';

    block.OutputPort(2).Dimensions  = 1; %MOMENTO DE INERCIA I
    block.OutputPort(2).DatatypeID  = 0;      % double
    block.OutputPort(2).Complexity  = 'Real';

    % Set output ports sampling mode:    
    block.OutputPort(1).SamplingMode = 'Sample';
    block.OutputPort(2).SamplingMode = 'Sample';

    % Register parameters:
    block.NumDialogPrms     = 3;
    block.DialogPrmsTunable = {'Nontunable','Nontunable','Nontunable'};

    % Register sample times:
    block.SampleTimes = [0 0]; %(Sets the S-Function sample time).

    % Specify the block simStateCompliance. The allowed values are:
    %    'UnknownSimState', < The default setting; warn and assume DefaultSimState
    %    'DefaultSimState', < Same sim state as a built-in block
    %    'HasNoSimState',   < No sim state
    %    'CustomSimState',  < Has GetSimState and SetSimState methods
    %    'DisallowSimState' < Error out when saving or restoring the model sim state
    block.SimStateCompliance = 'DefaultSimState';

    %% -----------------------------------------------------------------
    %% The MATLAB S-function uses an internal registry for all
    %% block methods. You should register all relevant methods
    %% (optional and required) as illustrated below. You may choose
    %% any suitable name for the methods and implement these methods
    %% as local functions within the same file. See comments
    %% provided for each function for more information.
    %% -----------------------------------------------------------------

    block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
%     block.RegBlockMethod('InitializeConditions', @InitializeConditions);
    block.RegBlockMethod('Start', @Start);
    block.RegBlockMethod('Outputs', @Outputs);     % Required
%     block.RegBlockMethod('Update', @Update);
%     block.RegBlockMethod('Derivatives', @Derivatives);
    block.RegBlockMethod('Terminate', @Terminate); % Required
end %setup

%%
%% PostPropagationSetup:
%%   Functionality    : Setup work areas and state variables. Can
%%                      also register run-time methods here
%%   Required         : No
%%   C MEX counterpart: mdlSetWorkWidths
function DoPostPropSetup(block)
    block.NumDworks = 4;

    block.Dwork(1).Name            = 'x2_clientID'; %Número identificador del cliente
    block.Dwork(1).Dimensions      = 1;
    block.Dwork(1).DatatypeID      = 0;      % double
    block.Dwork(1).Complexity      = 'Real'; % real
    block.Dwork(1).UsedAsDiscState = true;

    block.Dwork(2).Name            = 'x3_HA'; %Handles de las Articulaciónes
    block.Dwork(2).Dimensions      = 6;
    block.Dwork(2).DatatypeID      = 0;      % double
    block.Dwork(2).Complexity      = 'Real'; % real
    block.Dwork(2).UsedAsDiscState = true;

    block.Dwork(3).Name            = 'x4_HE'; %Handles de los Eslabónes
    block.Dwork(3).Dimensions      = 6;
    block.Dwork(3).DatatypeID      = 0;      % double
    block.Dwork(3).Complexity      = 'Real'; % real
    block.Dwork(3).UsedAsDiscState = true;
    
    block.Dwork(4).Name            = 'PA'; %Posiciones Articulares de CoppeliaSim
    block.Dwork(4).Dimensions      = 6;
    block.Dwork(4).DatatypeID      = 0;      % double
    block.Dwork(4).Complexity      = 'Real'; % real
    block.Dwork(4).UsedAsDiscState = true;
end %DoPostPropSetup

%%
%% Start:
%%   Functionality    : Called once at start of model execution. If you
%%                      have states that should be initialized once, this
%%                      is the place to do it.
%%   Required         : No
%%   C MEX counterpart: mdlStart
function Start(block)
    %CONEXIÓN CON COPPELIASIM:
    vrep = remApi('remoteApi'); %Using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); %Just in case, close all opened connections.
    clientID = vrep.simxStart('127.0.0.1',19000,true,true,5000,1);
    if (clientID > -1)
      disp('       [Conexión exitosa con CoppeliaSim. Conectado al servidor remoto de la API]');
      %block.OutputPort(1).Data = false; %Con 0 o false, la simulación de Simulink no se finaliza. Por
      %               defecto es siempre 0 o false [antes de iniciar la simulación] (yo no lo configuré).

      %__________________________________________________________________________________________________
      %OBTENCIÓN DE LOS HANDLES:
      %[Los HANDLES son simplemnte números enteros que utiliza CoppeliaSim para identificar a cada objeto
      %en la escena de simulación]
      %{
        ### simxGetObjectHandle:
        [number returnCode,number handle]=simxGetObjectHandle(number clientID,string objectPath,number
        operationMode)
        https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsMatlab.htm#simxGetObjectHandle
      %}
      %HANDLES DE LAS ARTICULACIONES:
      HA=[0,0,0,0,0,0]; %Inicialización del vector Handles de las Articulaciones
      [~,HA(1)] = vrep.simxGetObjectHandle(clientID,'Articulacion1_Base',vrep.simx_opmode_blocking);
      [~,HA(2)] = vrep.simxGetObjectHandle(clientID,'Articulacion2_Hombro',vrep.simx_opmode_blocking);
      [~,HA(3)] = vrep.simxGetObjectHandle(clientID,'Articulacion3_Codo',vrep.simx_opmode_blocking);
      [~,HA(4)] = vrep.simxGetObjectHandle(clientID,'Articulacion4_PrimerMuneca',vrep.simx_opmode_blocking);
      [~,HA(5)] = vrep.simxGetObjectHandle(clientID,'Articulacion5_SegundaMuneca',vrep.simx_opmode_blocking);
      [~,HA(6)] = vrep.simxGetObjectHandle(clientID,'Articulacion6_TerceraMuneca',vrep.simx_opmode_blocking);
%     disp(HA); % HA = [18    21    24    27    30    33]

      %HANDLES DE LOS ESLABONES:
      HE=[0,0,0,0,0,0]; %Inicialización del vector Handles de los Eslabónes
      [~,HE(1)] = vrep.simxGetObjectHandle(clientID,'Eslabon1_Hombro',vrep.simx_opmode_blocking);
      [~,HE(2)] = vrep.simxGetObjectHandle(clientID,'Eslabon2_Brazo',vrep.simx_opmode_blocking);
      [~,HE(3)] = vrep.simxGetObjectHandle(clientID,'Eslabon3_Antebrazo',vrep.simx_opmode_blocking);
      [~,HE(4)] = vrep.simxGetObjectHandle(clientID,'Eslabon4_PrimerMano',vrep.simx_opmode_blocking);
      [~,HE(5)] = vrep.simxGetObjectHandle(clientID,'Eslabon5_SegundaMano',vrep.simx_opmode_blocking);
      [~,HE(6)] = vrep.simxGetObjectHandle(clientID,'Eslabon6_visible',vrep.simx_opmode_blocking); %¡VISIBLE!
%     disp(HE); % HE = [19    22    25    28    31    34]

      %__________________________________________________________________________________________________
      %INICIO STREAMING DE LAS POSICIONES Y ORIENTACIONES DE LOS ESLABONES:

      %POSICIONES de los ESLABONES PE: [Posiciones Absolutas de las Tramas de los Eslabónes]
      PE = zeros(6,3); %Inicialización array PE
      %{
        ### simxGetObjectPosition:
        [number returnCode,array position]=simxGetObjectPosition(number clientID,number objectHandle,
        number relativeToObjectHandle,number operationMode)
        operationMode: a remote API function operation mode. Recommended operation modes for this
        function are simx_opmode_streaming (the first call) and simx_opmode_buffer (the following calls).
        [NOTA: el modo de operación "streaming" no puede no colocarse, sino después en el modo "buffer",
        todos los valores dan siempre cero].
        https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsMatlab.htm#simxGetObjectPosition
      %}
      for i=1:6
        [~,PE(i,:)] = vrep.simxGetObjectPosition(clientID,HE(i),-1,vrep.simx_opmode_streaming);
      end
      %ORIENTACIONES de los ESLABONES [utilizando QUATERNIONES] OE_Q:
      OE_Q = zeros(6,4); %Inicialización array OE_Q
      %{
        ### GetObjectQuaternion:
        [number returnCode,array quat]=simxGetObjectQuaternion(number clientID,number objectHandle,
        number relativeToObjectHandle,number operationMode)
        operationMode: a remote API function operation mode. Recommended operation modes for this
        function are simx_opmode_streaming (the first call) and simx_opmode_buffer (the following calls).
        [NOTA: el modo de operación "streaming" no puede no colocarse, sino después en el modo "buffer",
        todos los valores dan siempre cero].
        https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsMatlab.htm#simxGetObjectQuaternion
      %}
      for i=1:6
        [~,OE_Q(i,:)] = vrep.simxGetObjectQuaternion(clientID,HE(i),-1,vrep.simx_opmode_streaming);

      end
      %__________________________________________________________________________________________________
      %INICIO STREAMING DE LAS POSICIONES ARTICULARES:
      %(Esto es solo necesario para que se meta en el if para poder actualizar el valor de I).
      %{
        ### [number returnCode,number position]=simxGetJointPosition(number clientID,number jointHandle,
        number operationMode)
      %}
      PA = zeros(1,6); %Inicialización array PA
      for i=1:6
        [~,PA(i)] = vrep.simxGetJointPosition(clientID,HA(i),vrep.simx_opmode_streaming);
      end
      %__________________________________________________________________________________________________
      %PASAJE DE INFORMACIÓN ENTRE FUNCIONES:
      %[Para ello se hace uso de los vectores Dwork (Differential Work)]
      block.Dwork(1).Data = clientID;
      block.Dwork(2).Data = HA;
      block.Dwork(3).Data = HE;
      block.Dwork(4).Data = PA;
    else
      %__________________________________________________________________________________________________
      %TERMINACIÓN DE LA SIMULACIÓN EN CASO DE NO PODERSE CONECTAR CON COPPELIASIM:
      disp('[No se pudo establecer conexión entre MATLAB y CoppeliaSim. Error al intentar conectarse');
      disp(' con el servidor de la API. Asegúrese que la simulación de CoppeliaSim se encuentra corriendo]');
      vrep.delete(); %Call the destructor!

      block.OutputPort(1).Data = true; %Con un número distinto de 0 (como true==1) la simulación de Simulink
      % finaliza, mediante el uso del bloque STOP en Simulink.
    end
end %Start

%%
%% InitializeConditions:
%%   Functionality    : Called at the start of simulation and if it is
%%                      present in an enabled subsystem configured to reset
%%                      states, it will be called when the enabled subsystem
%%                      restarts execution to reset the states.
%%   Required         : No
%%   C MEX counterpart: mdlInitializeConditions
% function InitializeConditions(block)
%    %% Initialize Dwork:
%     block.Dwork(1).Data = ;
% end %InitializeConditions

%%
%% Update:
%%   Functionality    : Called to update discrete states
%%                      during simulation step
%%   Required         : No
%%   C MEX counterpart: mdlUpdate
% function Update(block)
%
% end %Update

%%
%% Outputs:
%%   Functionality    : Called to generate block outputs in
%%                      simulation step
%%   Required         : Yes
%%   C MEX counterpart: mdlOutputs
function Outputs(block)
  %% PASAJE DE POSICIONES ARTICULARES DE SIMULINK A COPPELIASIM:
%   fprintf('###################################################\n');
%   fprintf(' \n');
  vrep = remApi('remoteApi'); %Using the prototype file (remoteApiProto.m)
  clientID = block.Dwork(1).Data;

  q=block.InputPort(1).Data;   %[6x1] en rads
  q=q';                        %[1x6] en rads
  fprintf(' \n');
  
  HA = block.Dwork(2).Data;  
  vrep.simxSetJointTargetPosition(clientID,HA(1),q(1),vrep.simx_opmode_streaming);
  
  PA = block.Dwork(4).Data;
    
  q=round(q,2);
  PA=round(PA,2);
  if(q(2)~=PA(2)||q(3)~=PA(3)||q(4)~=PA(4)||q(5)~=PA(5)||q(6)~=PA(6))
%     fprintf('________________________________\n');
%     fprintf(' \n');
    
    for i=1:6
      [~,PA(i)] = vrep.simxGetJointPosition(clientID,HA(i),vrep.simx_opmode_buffer);
      vrep.simxSetJointTargetPosition(clientID,HA(i),q(i),vrep.simx_opmode_streaming);
    end                                                      %.simx_opmode_oneshot
                                                             %.simx_opmode_streaming
    block.Dwork(4).Data = PA;
    
    %% OBTENCIÓN DE LAS POSICIONES Y ORIENTACIONES DE LOS ESLABONES:

    %POSICIONES de los ESLABONES: [Posiciones Absolutas de las Tramas de los Eslabónes]
    PE = zeros(6,3); %Inicialización array PE
    HE = block.Dwork(3).Data;
%     fprintf('\nPOSICIONES Absolutas de las Tramas de los ESLABONES: \n');
    for i=1:6
      [~,PE(i,:)] = vrep.simxGetObjectPosition(clientID,HE(i),-1,vrep.simx_opmode_buffer);
%       fprintf('%.5f  ', PE(i,:));
%       fprintf('\n');
    end

%     %ORIENTACIONES de los ESLABONES [utilizando los ÁNGULOS DE EULER] OE_AE:
%     OE_AE = zeros(6,3); %Inicialización array OE_AE
%     HE = block.Dwork(3).Data;

    %ORIENTACIONES de los ESLABONES [utilizando QUATERNIONES] OE_Q:
    OE_Q = zeros(6,4); %Inicialización array OE_Q
%     fprintf('\nQUATERNIONES: \n');
    for i=1:6
      [~,OE_Q(i,:)] = vrep.simxGetObjectQuaternion(clientID,HE(i),-1,vrep.simx_opmode_streaming);
      OE_Q(i, [1,2,3,4]) = OE_Q(i, [4,1,2,3]); %CAMBIO DE ORDEN LOS QUATERNIONES DE [x y z w] A [w x y z]
%       fprintf('%.3f  ',OE_Q(i,:)); %Con 4 o más decimales, se ven las inexactitudes de la simulación
%       fprintf('\n');
    end

    %ORIENTACIONES de los ESLABONES [utilizando las MATRICES DE ROTACIÓN] OE_MR:
    OE_MR = zeros(6,3,3); %Inicialización array tridimensional OE_MR (contiene a 6 matrices de 3x3)
    %{
      https://la.mathworks.com/help/nav/ref/quat2rotm.html
      rotm = quat2rotm(quat)
      Quaternion vector must be of the form: quat=[w x y z] with w as the scalar number!->¡CoppeliaSim da
      el vector de cuaterniones de la forma: quat=[x y z w]!
    %}
    for i=1:6
      OE_MR(i,:,:) = quat2rotm(OE_Q(i,:));
    end
    %% PARÁMETROS DINÁMICOS DE LOS ESLABONES:
    %{
      %MASAS DE LOS ESLABÓNES:
      ME = [7.100, 12.700, 4.270, 2.000, 2.000, 0.365]; %[kg]

      %CENTROS DE MASA (RELATIVOS) DE LOS ESLABÓNES, [respecto a la Trama de la Malla en verde del eslabón]:
      CME_Mallas = [ 0.00000,  0.00587, -0.00355;   %Eslabón 1 Hombro
                     0.00000, -0.00076, -0.03733;   %Eslabón 2 Brazo
                     0.00000,  0.00388, -0.01720;   %Eslabón 3 Antebrazo
                     0.00000, -0.00304,  0.00213;   %Eslabón 4 Primer Mano
                     0.00000,  0.00213, -0.00302;   %Eslabón 5 Segunda Mano
                     0.00000, -0.00375, -0.00018];  %Eslabón 6 VISIBLE (Tercera Mano)

      %MOMENTOS Y PRODUCTOS DE INERCIA [kg.m^2] DE CADA ESLABÓN (respecto al COM de cada eslabón):
      %     Ixx 	  Iyy	    Izz	      Ixy      Iyz      Izx
      IE = [0.03529 0.03408 0.02156   0.00000 -0.00425  0.00000;  %Eslabón 1 Hombro
            0.77068 0.76943 0.02814   0.00000 -0.01561  0.00000;  %Eslabón 2 Brazo
            0.30928 0.30646 0.01014   0.00000 +0.00916  0.00000;  %Eslabón 3 Antebrazo
            0.00296 0.00258 0.00222   0.00000 -0.00024  0.00000;  %Eslabón 4 Primer Mano
            0.00296 0.00222 0.00258   0.00000 -0.00024  0.00000;  %Eslabón 5 Segunda Mano
            0.00040 0.00034 0.00041   0.00000  0.00000  0.00000]; %Eslabón 6 VISIBLE (Tercera Mano)
    %}
    ME = block.DialogPrm(1).Data;  %Access the dialog box parameters
    CME_Mallas = block.DialogPrm(2).Data;
    IE = block.DialogPrm(3).Data;
    
    %% CÁLCULO DEL MOMENTO DE INERCIA I (respecto al eje Z de la Trama Mundo):

    %DISTANCIA [EN LA DIRECCIÓN 'X' de la Trama Mundo] desde el origen de coordenadas de la Trama Malla
    %del eslabón, hasta su Centro de Masa:
    %DisX_Mallas_CM = (aporte de la coordenada x del CME_Mallas, a la coordenada X) + ...
    %               (aporte de la coordenada y del CME_Mallas, a la coordenada X) + ...
    %               (aporte de la coordenada z del CME_Mallas, a la coordenada X);
    DisX_Mallas_CM = zeros(6,1);
    for i=1:6
      DisX_Mallas_CM(i) = CME_Mallas(i,1)*OE_MR(i,1,1) + CME_Mallas(i,2)*OE_MR(i,1,2) + CME_Mallas(i,3)*OE_MR(i,1,3);
    end

    %DISTANCIA [EN LA DIRECCIÓN 'Y' de la Trama Mundo] desde el origen de coordenadas de la Trama Mallas
    %del eslabón, hasta su Centro de Masa:
    %DisY_Mallas_CM = (aporte de la coordenada x del CME_Mallas, a la coordenada Y) + ...
    %               (aporte de la coordenada y del CME_Mallas, a la coordenada Y) + ...
    %               (aporte de la coordenada z del CME_Mallas, a la coordenada Y);
    DisY_Mallas_CM = zeros(6,1);
    for i=1:6
      DisY_Mallas_CM(i) = CME_Mallas(i,1)*OE_MR(i,2,1) + CME_Mallas(i,2)*OE_MR(i,2,2) + CME_Mallas(i,3)*OE_MR(i,2,3);
    end

    %CENTROS DE MASA (ABSOLUTOS) DE LOS ESLABÓNES, [respecto a la Trama del Mundo]:
    CME_Mundo = zeros(6,2);
    for i=1:6
      CME_Mundo(i,:) = [ PE(i,1)+DisX_Mallas_CM(i), PE(i,2)+DisY_Mallas_CM(i) ];%(Coordenada Z No se necesita)
    end



    %CÁLCULO DEL MOMENTO DE INERCIA DE UN CUERPO CON RESPECTO A UN EJE ARBITRARIO OL:
    IE_OL = zeros(6,1);
    for i=1:6
      IE_OL(i) = IE(i,1)*OE_MR(i,3,1)^2 + IE(i,2)*OE_MR(i,3,2)^2 + IE(i,3)*OE_MR(i,3,3)^2 - ...
                 2*IE(i,4)*OE_MR(i,3,1)*OE_MR(i,3,2) - 2*IE(i,5)*OE_MR(i,3,2)*OE_MR(i,3,3) - ...
                 2*IE(i,6)*OE_MR(i,3,3)*OE_MR(i,3,1);
    end

    %MOMENTO DE INERCIA (respecto al eje Z de la Trama Mundo):
    I = IE_OL(1) + ME(1)*( CME_Mundo(1,1)^2 + CME_Mundo(1,2)^2 ) +...
        IE_OL(2) + ME(2)*( CME_Mundo(2,1)^2 + CME_Mundo(2,2)^2 ) +...
        IE_OL(3) + ME(3)*( CME_Mundo(3,1)^2 + CME_Mundo(3,2)^2 ) +...
        IE_OL(4) + ME(4)*( CME_Mundo(4,1)^2 + CME_Mundo(4,2)^2 ) +...
        IE_OL(5) + ME(5)*( CME_Mundo(5,1)^2 + CME_Mundo(5,2)^2 ) +...
        IE_OL(6) + ME(6)*( CME_Mundo(6,1)^2 + CME_Mundo(6,2)^2 );
    I = round(I,4); %Se redondea a cuatro decimales.
    block.OutputPort(2).Data = I;    
  end
end %Outputs

%%
%% Derivatives:
%%   Functionality    : Called to update derivatives of
%%                      continuous states during simulation step
%%   Required         : No
%%   C MEX counterpart: mdlDerivatives
% function Derivatives(block)
% end %Derivatives

%%
%% Terminate:
%%   Functionality    : Called at the end of simulation for cleanup
%%   Required         : Yes
%%   C MEX counterpart: mdlTerminate
function Terminate(block)
    vrep = remApi('remoteApi'); %Using the prototype file (remoteApiProto.m)
    vrep.delete(); %Call the destructor!
end %Terminate