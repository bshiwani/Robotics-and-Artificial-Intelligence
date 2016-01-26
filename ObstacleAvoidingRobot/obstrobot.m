function varargout = obstrobot(varargin)
%This Program Starts with callback controls functions for the simulator GUI
%window. The callback functions for each option are defined.

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @obstrobot_OpeningFcn, ...
                   'gui_OutputFcn',  @obstrobot_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT
end

% --- Executes just before obstrobot is made visible.
function obstrobot_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to obstrobot (see VARARGIN)

% Choose default command line output for obstrobot
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes obstrobot wait for user response (see UIRESUME)
% uiwait(handles.figure1);

end
% --- Outputs from this function are returned to the command line.
function varargout = obstrobot_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

end
%% Under this section callback functions of edit are defined.
%  These functions are used to display the position and rotation parameters
%  in the simulator window.
%  The Functions are linked to following outputs
%       edit2 = Relative Direction
%       edit5 = Absolute Position X Coordinate  
%       edit8 = Absolute Position Y Coordinate
%       edit6 = Relative Position X Coordinate 
%       edit7 = Relative Position Y Coordinate

function edit2_Callback(hObject, eventdata, handles)
end
function edit2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

function edit5_Callback(hObject, eventdata, handles)
end
function edit5_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

function edit6_Callback(hObject, eventdata, handles)
end
function edit6_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

function edit7_Callback(hObject, eventdata, handles)
end
function edit7_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

function edit8_Callback(hObject, eventdata, handles)
end
function edit8_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

%% Under this section callback functions of buttons are defined.
% In this section the callback functions for toggle buttons for Display 
% Trace, Record, Start and Stop are defined. The Buttons are created for
% controlling the display of coordinates and rotations as well, However the
% functionality is not implemented yet.

% The Callback functions are linked to following buttons
%       togglebutton2 = Show Trace/Disable Show Trace
%       togglebutton3 = Record Video/Pause Recording
%       togglebutton4 = Not implemeted yet
%       togglebutton5 = Not implemeted yet
%       togglebutton6 = Not implemeted yet
%       togglebutton8 = Start/Restart the simulator
%       togglebutton9 = Stop Simulator

% --- Executes on button press in togglebutton2.
function togglebutton2_Callback(hObject, eventdata, handles)
isPushed = get(hObject, 'Value');
if isPushed
    set(hObject,'string','Trace Displayed','ForegroundColor','red','enable','on');
else
    set(hObject, 'String' , 'Show Trace');
end
end
% --- Executes on button press in togglebutton3.
function togglebutton3_Callback(hObject, eventdata, handles)
isPushed = get(hObject, 'Value');
if isPushed
    set(hObject,'string','Pause','ForegroundColor','red','enable','on');
else
    set(hObject, 'String' , 'Record Video');
end
end
% --- Executes on button press in togglebutton9.
function togglebutton9_Callback(hObject, eventdata, handles)
end
% --- Executes on button press in togglebutton8.

%% The actual code for obstacle avoidance
% The code for robot simulation is implemeted under togglebutton8 callback 
% function. 
function togglebutton8_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of togglebutton8

% Reset the Record and Show Trace buttons when Start or Restart button is
% pressed.
isPushed = get(hObject, 'Value');
if isPushed
    set(hObject,'string','Restart','ForegroundColor','red','enable','on');
else
    set(hObject, 'String' , 'Start');
end
set(handles.togglebutton2,'Value',0);
set(handles.togglebutton3,'Value',0);
set(handles.togglebutton2, 'String' , 'Show Trace');
set(handles.togglebutton3, 'String' , 'Record Video');

%% Setting Parameters for robot and robot world map
%Initialize a grid 400x400
%Add obstacles
%Create occupancy grid
%Initialize robot positions
%Possible movements 8 directions
%Check in 5 next grids in all directions

% Set the World size in terms of nxn grid 
    n=60;
    
% Select Total number of obstacles in the grid (Obstacles may overlap)
    obstacle_num= 6; %Size of obstacle is proportional to grid size

% Initialize the robot world, and starting coordinates of robot
    [world,init]=init_world(n,obstacle_num);
    
% Set current location as initial coordinates
    loc=init;

% Start path tracing
    path=loc;
    
% Draw the Initial World
    h = handles.figure1;
    drawWorld(world, loc, h);
    
% Set the radius(Euclidian distance) for detection in 8 directions
    rad=min(2,ceil(n/20));
    
% Set number of frames to be recorded for the video, frame parameter
    nframes=1000;
    mov=1;
% Initialize current direction (dummy)
    dir_old = 'D';

% Initialize the count for number of steps in same direction
    count =0;
    
% Initialize the base rotation parameter
    rot_old=0;
% Rotation angles are computed anti-clockwise starting from +ve x-axis as
% 0 degrees. Relative angles are also computed accordingly.
      
% Flags to implement exclusive behavior of togglebuttons 2,3
    out=0;
    out1=0;
    
% The loop for robot motion starts    
while true
% Check if stop button is pressed. When pressed reset the toggle buttons
% and exit from the loop
    if get(handles.togglebutton9,'Value')==1
        set(handles.togglebutton2,'Value',0);
        set(handles.togglebutton3,'Value',0);
        set(handles.togglebutton2, 'String' , 'Show Trace');
        set(handles.togglebutton3, 'String' , 'Record Video');
        break;
    end

% Draw the world as well as initial position of robot 
    h = handles.figure1;
    drawWorld(world, loc, h);
    plot(loc(2), loc(1), 'go')

% Sense for obstacles and determine the direction for next step. Also keep 
% track of count of repeated steps in same direction.  
    [direction,count]=sense(world,rad,n,loc,dir_old,count);

% Save old direction to continue moving in case no new obstacle is within
% the range
    dir_old = direction;

% Move as per sensor results and determine new location and rotation (with
% respect to 0 degress axis)     
    [loc,rot]=move(loc,direction);

% Find the relative rotation and print it to GUI window
    rel_rot=rot-rot_old;
    string5 = sprintf('Rel Rot = %.3f', rel_rot);
    set(handles.edit2, 'String', string5);
    
% Save old rotation to find relative rotation of next step
    rot_old=rot;
    
% Display the absolute position returned by move function to GUI window
    string1 = sprintf('Abs X = %.3f', loc(1));
    set(handles.edit5, 'String', string1);
    string2 = sprintf('Abs Y = %.3f', loc(2));
    set(handles.edit8, 'String', string2);
    
% Path matrix stores all the positions traversed by robot. Determiine the
% relative position of robot using path matrix and current location
    relX=loc(1)-path(end,1);
    relY=loc(2)-path(end,2);
  
% Display the relative coordinates to simulator GUI window
    string3 = sprintf('Rel X = %.3f', relX);
    set(handles.edit6, 'String', string3);
    string4 = sprintf('Rel Y = %.3f', relY);
    set(handles.edit7, 'String', string4);
    
% Update the path matrix to include current step
    path=[path;loc];
    
% Check if show trace button is pressed. If pressed change the button
% status accordingly and pass the flag(out) to drawPath function to include
% the Trace
    press=get(handles.togglebutton2,'Value');
    if get(handles.togglebutton2,'Value')==1
        out = xor(out,press);
        if out==1
            set(handles.togglebutton2,'string','Trace Displayed','ForegroundColor','red','enable','on');
        else
            set(handles.togglebutton2, 'String' , 'Show Trace');
        end
        set(handles.togglebutton2,'Value',0);
    end
    
% Call the draw path function to display new postion of robot on map 
    drawPath(path, h,out);
 
% Pause(if required)
    %pause(0.10)
    record=get(handles.togglebutton3,'Value');

% Check if Record Video button is pressed. If pressed change the button
% status accordingly and pass the flag(out1) to save frames to record a
% video of robot motion.
    if get(handles.togglebutton3,'Value')==1
        out1 = xor(out1,record);
        if out1==1
            set(handles.togglebutton3,'string','Pause','ForegroundColor','red','enable','on');
        else
            set(handles.togglebutton3, 'String' , 'Record Video');
        end
        set(handles.togglebutton3,'Value',0);
    end
    
% Check the flag(out1) status and record video. The video can be paused and
% continued. However it will only save 'nframes' to make video 
     if out1 ==1
         if mov<=nframes
             M(mov) = getframe;
             mov=mov+1;
         else
             % Finally, play the movie once
             %movie(M,1)
             % Save movie in avi format
             movie2avi(M,'robot.avi')
         end
     end
end
% After Stop is pressed check and save the video even if the number of
% frames are less than 'nframes' 
   if mov<=nframes
       %movie(M,1)
       % Save movie in avi format
       movie2avi(M,'robot.avi') 
   end

end
%% The function used to Initialize the world grid and robot
function [world,init]=init_world(n,obstacle_num)
% Create a world of desired dimentions
    world=zeros(n,n);

%Put boundaries as 1(obstacles) so that robot doesn't overshoot
    world(1,:)=1;
    world(n,:)=1;
    world(:,1)=1;
    world(:,n)=1;

%Create Obstacles
% The obstacles are created be choosing a submatrix from world matrix and
% putting random seeds(pixels) for obstacles in that. Later these seeds
% grow to make obstacles relative to the size of world grid.
% Submatrix is selected instead of full world such that after growing the 
% obstacles remain within the world grid and dont overshoot. 
    dim=floor(n-2-n/5);
    obst_area=zeros(dim,dim);
    obst_area(ind2sub([dim,dim],randperm(dim*dim,obstacle_num)))=1;

% Adding the rows back after creating random seeds
    border_cell=floor(1+n/10);
    obst_area=[world(1:border_cell,border_cell+1:n-border_cell); obst_area; world(n-border_cell+1:n,border_cell+1:n-border_cell)];

% Adding the columns back after creating random seeds
    obst_area=[world(:,1:border_cell) obst_area world(:,n-border_cell:n)];

% Create another matrix to store grown obstacles
    obst_area1=obst_area;
    
% Code to grow the random seeds to obstacles
    for i=border_cell+1:n-border_cell
        for j=border_cell+1:n-border_cell
            if obst_area(i,j)==1
                for k=-n/10:n/10
                    for l=-n/10:n/10
                        obst_area1(i+k,j+l)=1;
                    end
                end
            end
        end
    end

% Save the Final Matrix 
    obst_area=obst_area1;  
    
% Find an empty space the map to launch the robot
    init=[0,0];
    for i=1:n
        for j=1:n
            if obst_area(i,j)==0
                init=[i,j];
                break
            end
        end
    end

% Return world map and initial robot coordinates
    world=obst_area;
end
%% Function to Draw World and the current location of robot
function drawWorld(world, loc, h)
    
    world = 1-world;
    world(loc(1),loc(2)) = 0.66;
       
    figure(h)
    imagesc(world); colormap(gray); axis off; hold on
end
%% Function to sense the obstacles
function [direction,count] = sense(world,rad,n,loc,dir_old,count)
    % Check in radius of 8 sensors range. Start from neighborhood pixels
    % and proceed to sense in complete radius
    % If obstacle is detected in any direction return the code for that
    % direction, If there is no obstacle detected then keep following the
    % previous direction till the count reaches desired values, then
    % randomize the robot direction(and restart the count).
    for i=1:rad
        if world(loc(1)+i,loc(2)) ==1
            check = [-1,0];
            direction = 'S';
            break
        elseif world(loc(1),loc(2)+i) ==1
            check = [0,1];
            direction = 'E';
            break
        elseif world(loc(1)-i,loc(2))==1
            check = [1,0];
            direction = 'N';
            break
        elseif world(loc(1),loc(2)-i)==1
            check = [0,-1];
            direction = 'W';
            break
        elseif world(loc(1)+i,loc(2)+i) ==1
            check = [-1,1];
            %direction = 'SE';
            direction = 'B';
            break
        elseif world(loc(1)-i,loc(2)+i) ==1
            check = [1,1];
            %direction = 'NE';
            direction = 'H';
            break
        elseif world(loc(1)+i,loc(2)-i)==1
            check = [-1,-1];
            %direction = 'SW';
            direction = 'A';
            break
        elseif world(loc(1)-i,loc(2)-i)==1
            check = [1,-1];
            %direction = 'NW';
            direction = 'K';
            break
        elseif count >=n/5;
                s=['B' 'H' 'A' 'K' 'N' 'S' 'E' 'W'];
                direction =s(randi(8,1,1));
                count =0;
        else
            count = count+1;
            direction = dir_old;
        end
    end
end
%% This function determines the next robot location and rotation
% In this function the next location of the robot and rotation angle with
% respect to base axis is determined based on current location and sensor
% output code
function [loc,rot]=move(loc,direction)
    switch(direction)
        case 'E'
            loc=[loc(1),loc(2)-1];
            rot=180;
        case 'W'
            loc=[loc(1),loc(2)+1];
            rot=0;
        case 'N'
            loc=[loc(1)+1,loc(2)];
            rot=270;
        case 'S'
            loc=[loc(1)-1,loc(2)];
            rot=90;
        case 'H'
            loc=[loc(1)+1,loc(2)-1];
            rot=215;
        case 'A'
            loc=[loc(1)-1,loc(2)+1];
            rot=45;
        case 'K'
            loc=[loc(1)+1,loc(2)+1];
            rot=315;
        case 'B'
            loc=[loc(1)-1,loc(2)-1];
            rot=135;
    end
end
%% Function to draw the path traversed by robot
% This function plots the path traversed by robot, and its current position
function drawPath(path, g,press)
    
    figure(g); hold on
    if press==1
        plot(path(:,2),  path(:,1),   'co', 'LineWidth', 2)
    end
    plot(path(1,2),  path(1,1),   'gs', 'LineWidth', 4)
    plot(path(end,2),path(end,1), 'rs', 'LineWidth', 4)
    drawnow
    hold off
end
