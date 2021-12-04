

function varargout = HohmannTransfer(varargin)
% HOHMANNTRANSFER MATLAB code for HohmannTransfer.fig
%      HOHMANNTRANSFER, by itself, creates a new HOHMANNTRANSFER or raises the existing
%      singleton*.
%
%      H = HOHMANNTRANSFER returns the handle to a new HOHMANNTRANSFER or the handle to
%      the existing singleton*.
%
%      HOHMANNTRANSFER('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in HOHMANNTRANSFER.M with the given input arguments.
%
%      HOHMANNTRANSFER('Property','Value',...) creates a new HOHMANNTRANSFER or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before HohmannTransfer_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to HohmannTransfer_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help HohmannTransfer

% Last Modified by GUIDE v2.5 02-Dec-2021 00:50:30

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @HohmannTransfer_OpeningFcn, ...
                   'gui_OutputFcn',  @HohmannTransfer_OutputFcn, ...
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


% --- Executes just before HohmannTransfer is made visible.
function HohmannTransfer_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to HohmannTransfer (see VARARGIN)

handles.plotdata = NaN;

% plot earth
% ------------------------------------------------------------
handles.R = 6400000; %[m]
[X, Y ,Z] = ellipsoid(0,0,0,handles.R,handles.R,handles.R);
original = imread('map.jpg');
imshow(original);
surf(X, Y, -Z);
axis([-10000000 10000000 -10000000 10000000 -10000000 10000000]);
h=findobj('Type','surface');
set(h,'CData',original,'FaceColor','texturemap','edgecolor','none')

enableDefaultInteractivity(gca); % enable user control axis
hold on;

title('Hohmann Transfer');
xlabel('[m]');
ylabel('[m]');
zlabel('[m]');
legend('Earth');
% ------------------------------------------------------------

% Choose default command line output for HohmannTransfer
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);
% UIWAIT makes HohmannTransfer wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = HohmannTransfer_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function T_initial_output_Callback(hObject, eventdata, handles)
% hObject    handle to T_initial_output (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of T_initial_output as text
%        str2double(get(hObject,'String')) returns contents of T_initial_output as a double


% --- Executes during object creation, after setting all properties.
function T_initial_output_CreateFcn(hObject, eventdata, handles)
% hObject    handle to T_initial_output (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function T_final_output_Callback(hObject, eventdata, handles)
% hObject    handle to T_final_output (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of T_final_output as text
%        str2double(get(hObject,'String')) returns contents of T_final_output as a double


% --- Executes during object creation, after setting all properties.
function T_final_output_CreateFcn(hObject, eventdata, handles)
% hObject    handle to T_final_output (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function v_initial_output_Callback(hObject, eventdata, handles)
% hObject    handle to v_initial_output (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of v_initial_output as text
%        str2double(get(hObject,'String')) returns contents of v_initial_output as a double


% --- Executes during object creation, after setting all properties.
function v_initial_output_CreateFcn(hObject, eventdata, handles)
% hObject    handle to v_initial_output (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function v_final_output_Callback(hObject, eventdata, handles)
% hObject    handle to v_final_output (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of v_final_output as text
%        str2double(get(hObject,'String')) returns contents of v_final_output as a double


% --- Executes during object creation, after setting all properties.
function v_final_output_CreateFcn(hObject, eventdata, handles)
% hObject    handle to v_final_output (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function v_perigee_output_Callback(hObject, eventdata, handles)
% hObject    handle to v_perigee_output (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of v_perigee_output as text
%        str2double(get(hObject,'String')) returns contents of v_perigee_output as a double


% --- Executes during object creation, after setting all properties.
function v_perigee_output_CreateFcn(hObject, eventdata, handles)
% hObject    handle to v_perigee_output (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function v_apogee_output_Callback(hObject, eventdata, handles)
% hObject    handle to v_apogee_output (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of v_apogee_output as text
%        str2double(get(hObject,'String')) returns contents of v_apogee_output as a double


% --- Executes during object creation, after setting all properties.
function v_apogee_output_CreateFcn(hObject, eventdata, handles)
% hObject    handle to v_apogee_output (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function r_initial_input_Callback(hObject, eventdata, handles)
% hObject    handle to r_initial_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.r_initial = 1000 * str2double(get(hObject,'String'));

% Hints: get(hObject,'String') returns contents of r_initial_input as text
%        str2double(get(hObject,'String')) returns contents of r_initial_input as a double
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function r_initial_input_CreateFcn(hObject, eventdata, handles)
% hObject    handle to r_initial_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function r_final_input_Callback(hObject, eventdata, handles)
% hObject    handle to r_final_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.r_final = 1000 * str2double(get(hObject,'String'));

% Hints: get(hObject,'String') returns contents of r_final_input as text
%        str2double(get(hObject,'String')) returns contents of r_final_input as a double
% Update handles structure
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function r_final_input_CreateFcn(hObject, eventdata, handles)
% hObject    handle to r_final_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function mass_input_Callback(hObject, eventdata, handles)
% hObject    handle to mass_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.mass = str2double(get(hObject,'String'));

% Hints: get(hObject,'String') returns contents of mass_input as text
%        str2double(get(hObject,'String')) returns contents of mass_input as a double
% Update handles structure
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function mass_input_CreateFcn(hObject, eventdata, handles)
% hObject    handle to mass_input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in play.
function play_Callback(hObject, eventdata, handles)
% hObject    handle to play (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if ~isnan(handles.plotdata)
    delete(handles.orbit1);
    delete(handles.orbit2);
    delete(handles.orbit3);
end

m = handles.mass;
r_initial = handles.r_initial + handles.R;
r_final = handles.r_final + handles.R;

G = 6.67408e-11;
M = 5.9722e24;
mu = G*M;

v_initial = sqrt(mu/r_initial);
v_perigee = sqrt(2*mu/r_initial-mu/((r_initial+r_final)/2));
v_apogee = sqrt(2*mu/r_final-mu/((r_initial+r_final)/2));
v_final = sqrt(mu/r_final);

T_initial = 2*pi*sqrt((r_initial^3)/mu);
T_final = 2*pi*sqrt((r_final^3)/mu);

set(handles.v_apogee_output,'String',sprintf('%.3f',v_apogee));
set(handles.v_perigee_output,'String',sprintf('%.3f',v_perigee));
set(handles.v_initial_output,'String',sprintf('%.3f',v_initial));
set(handles.v_final_output,'String',sprintf('%.3f',v_final));
set(handles.T_initial_output,'String',sprintf('%.2f',T_initial/60));
set(handles.T_final_output,'String',sprintf('%.2f',T_final/60));


% Orbit 1
% ------------------------------------------------------------

r = [r_initial, 0, 0];
v = [0, v_initial, 0];
spendtime = 0;
dt = 1;
forplot = r;

while(spendtime < T_initial)
    
    a = -G*M/norm(r)^2/norm(r)*r;
    r = r + v *dt + a*(dt^2)/2;
    v = v + a*dt;
    spendtime = spendtime + dt;
    forplot = [forplot;r];
    
end

handles.orbit1 = plot3(forplot(:,1),forplot(:,2),forplot(:,3),'b');


% Orbit 2
% ------------------------------------------------------------

forplot = r;
dt = 1;
theta = 0;
v = [0, v_perigee, 0];
r_perigee = r;

while(theta <= 3.1414)
    
    a = -G*M/norm(r)^2/norm(r)*r;
    r = r + v *dt + a*(dt^2)/2;
    v = v + a*dt;
    forplot = [forplot;r];
    theta = acos(dot(r_perigee,r)/(norm(r_perigee)*norm(r)));

end

handles.orbit2 = plot3(forplot(:,1),forplot(:,2),forplot(:,3),'r');


% Orbit 3
% ------------------------------------------------------------ 

v = [0, -v_final, 0];
dt = 1;
forplot = r;
spendtime = 0;

while(spendtime <= T_final)
    
    a = -G*M/norm(r)^2/norm(r)*r;
    r = r + v *dt + a*(dt^2)/2;
    v = v + a*dt;
    forplot = [forplot;r];
    spendtime = spendtime + dt;
    
end

handles.orbit3 = plot3(forplot(:,1),forplot(:,2),forplot(:,3),'k');

axis([-norm(r)*4/3 norm(r)*4/3 -norm(r)*4/3 norm(r)*4/3 -norm(r)*4/3 norm(r)*4/3]);
legend('Earth','1st Orbit','2nd Orbit','3rd Orbit');

handles.plotdata = forplot;

% Update handles structure
guidata(hObject, handles);


% --- Executes on mouse press over axes background.
function axes1_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
