function varargout = A2(varargin)
% A2 MATLAB code for A2.fig
%      A2, by itself, creates a new A2 or raises the existing
%      singleton*.
%
%      H = A2 returns the handle to a new A2 or the handle to
%      the existing singleton*.
%
%      A2('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in A2.M with the given input arguments.
%
%      A2('Property','Value',...) creates a new A2 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before A2_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to A2_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help A2

% Last Modified by GUIDE v2.5 17-Oct-2018 14:33:44

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @A2_OpeningFcn, ...
                   'gui_OutputFcn',  @A2_OutputFcn, ...
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


% --- Executes just before A2 is made visible.
function A2_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to A2 (see VARARGIN)

% Choose default command line output for A2
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes A2 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = A2_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
      
%loading image
        global im 
        [path,user_cance]=imgetfile();
        if user_cance
            msgbox(sprintf('Error'),'Error','Error');
            return
        end

        im=imread(path);
        axes(handles.axes1);
        imshow(im);


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)   
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global im im2 blur
i = im;
bk = im2;

iR=i(:,:,1);
iG=i(:,:,2);
iB=i(:,:,3);

[h, w] = size(iR);
[hk, wk] = size(bk);

scale1 = h+hk-1;
scale2 = w+wk-1;

S=0;

for i=1:hk
    for j=1:wk
    S= S+bk(i,j);
    end
end

iR_pad = zeros(scale1,scale2);
iG_pad = zeros(scale1,scale2);
iB_pad = zeros(scale1,scale2);
bks_pad = zeros(scale1,scale2);

iR_pad(1:h,1:w) = iR;
iG_pad(1:h,1:w) = iG;
iB_pad(1:h,1:w) = iB;

bks_pad(1:hk,1:wk) = bk;

fftR= fftshift(fft2(iR_pad));
fftG= fftshift(fft2(iG_pad));
fftB= fftshift(fft2(iB_pad));

FK = fftshift(fft2(bks_pad));

RB = fftR.*FK;
GB = fftG.*FK;
BB = fftB.*FK;

iRB=ifft2(ifftshift((RB)));
iGB=ifft2(ifftshift((GB)));
iBB=ifft2(ifftshift((BB)));

bR = zeros(h,w);
bR= iRB(1:h,1:w);

bG = zeros(h,w);
bG= iGB(1:h,1:w);

bB = zeros(h,w);
bB= iBB(1:h,1:w);

finalblur_RGB= cat(3,bR,bG,bB);


bf=real(finalblur_RGB);
D=abs((255/max(bf(:))).*bf);
blur =D;
axes(handles.axes3);
imshow (uint8(D)),title('blurred');


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global blur im2
i = blur;
bk = im2;

iR=i(:,:,1);
iG=i(:,:,2);
iB=i(:,:,3);

[h, w] = size(iR);
[hk, wk] = size(bk);

scale1 = h+hk-1;
scale2 = w+wk-1;

iR_pad = zeros(scale1,scale2);
iG_pad = zeros(scale1,scale2);
iB_pad = zeros(scale1,scale2);
bks_pad = zeros(scale1,scale2);

iR_pad(1:h,1:w) = iR;
iG_pad(1:h,1:w) = iG;
iB_pad(1:h,1:w) = iB;

bks_pad(1:hk,1:wk) = bk;

fftR= fftshift(fft2(iR_pad));
fftG= fftshift(fft2(iG_pad));
fftB= fftshift(fft2(iB_pad));

FK = fftshift(fft2(bks_pad));

for p=1:scale1
    for q=1:scale2  
        if abs(FK(p,q))<1000
            FK(p,q)=1000;
        end
    end
end


RB = fftR./FK;
GB = fftG./FK;
BB = fftB./FK;

iRB=ifft2(ifftshift((RB)));
iGB=ifft2(ifftshift((GB)));
iBB=ifft2(ifftshift((BB)));

bR = zeros(h,w);
bR= iRB(1:h,1:w);

bG = zeros(h,w);
bG= iGB(1:h,1:w);

bB = zeros(h,w);
bB= iBB(1:h,1:w);

finalblur_RGB= cat(3,bR,bG,bB);
finalblur_RGB=abs((255/max(finalblur_RGB(:))).*finalblur_RGB);

bf=real(finalblur_RGB);
%bf= bf^(2.2);
D=abs((255/max(bf(:))).*bf);
axes(handles.axes3);
imshow (uint8(bf)); %deblurred

% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
  global im2
        [path,user_cance]=imgetfile();
        if user_cance
            msgbox(sprintf('Error'),'Error','Error');
            return
        end

        im2=imread(path);
        axes(handles.axes2);
        imshow(im2);


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
%truncated

global im3 blur im2
im3 = blur;
 i= im3;
 bk = im2;
 
iR=i(:,:,1);
iG=i(:,:,2);
iB=i(:,:,3);

[h, w] = size(iR);
[hk, wk] = size(bk);

scale1 = h+hk-1;
scale2 = w+wk-1;

iR_pad = zeros(scale1,scale2);
iG_pad = zeros(scale1,scale2);
iB_pad = zeros(scale1,scale2);
bks_pad = zeros(scale1,scale2);

iR_pad(1:h,1:w) = iR;
iG_pad(1:h,1:w) = iG;
iB_pad(1:h,1:w) = iB;
bks_pad(1:hk,1:wk) = bk;

fftR= fftshift(fft2(iR_pad));
fftG= fftshift(fft2(iG_pad));
fftB= fftshift(fft2(iB_pad));

FK = fftshift(fft2(bks_pad));

%FKi = fftshift(fft2(bks_pad));

r1=400*get(hObject,'Value');
r2=r1;

FKn = ones(scale1,scale2);

p1=((scale1-r1)/2)+1;
p2=(scale1+r1)/2;
q1=((scale2-r2)/2)+1;
q2=(scale2+r2)/2;

for p = p1:p2
    for q = q1:q2  
         FKn(p,q)=FK(p,q);
    end
end

FKn(abs(FKn) < 1500) = 1500;

RB = fftR./FKn;
GB = fftG./FKn;
BB = fftB./FKn;

iRB=ifft2(ifftshift((RB)));
iGB=ifft2(ifftshift((GB)));
iBB=ifft2(ifftshift((BB)));

bR = zeros(h,w);
bR= iRB(1:h,1:w);

bG = zeros(h,w);
bG= iGB(1:h,1:w);

bB = zeros(h,w);
bB= iBB(1:h,1:w);

finalblur_RGB= cat(3,bR,bG,bB);
finalblur_RGB=(abs(255/max(finalblur_RGB(:))).*finalblur_RGB);

bf=real(finalblur_RGB);

D=abs((255/max(bf(:))).*bf);

img=bf;
img=double(img(:));
ima=max(img(:));
imi=min(img(:));
mse=std(img(:));
kout = (10*log10((ima^2)./mse));

txt1=num2str(kout);
set(handles.edit1,'String',txt1);

%[s,v]=ssim(bf,blur);
%
X = bf;
%figure, imshow(X);
X=rgb2gray(X);
%figure, imshow(X);
Y =blur;
%figure, imshow(Y);
Y=rgb2gray(Y);
%figure, imshow(Y);
Ux = mean(mean(X));
Uy = mean(mean(Y));
sigma_X = mean(mean((X-Ux).^2));
sigma_Y = mean(mean((Y-Uy).^2));
sigma_XY = mean(mean((X-Ux).*(Y-Uy)));
k1 = 0.01;
k2 = 0.03;
l =255;
c1 =k1*l;
c2 = k2*l;
ans= 0.40*( (2*Ux*Uy+c1)*(2*sigma_XY+c2) )/( (Ux^2+Uy^2+c1)*(sigma_X+sigma_Y+c2) );

%
txt3=num2str(ans);
set(handles.edit2,'String',txt3);

 axes(handles.axes3);
        imshow(uint8(bf));

% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


%Weiner
 global im3 blur im2
 im3 = blur;
 i= im3;
 bk = im2;

iR=i(:,:,1);
iG=i(:,:,2);
iB=i(:,:,3);

[h, w] = size(iR);
[hk, wk] = size(bk);

scale1 = h+hk-1;
scale2 = w+wk-1;

iR=i(:,:,1);
iG=i(:,:,2);
iB=i(:,:,3);

[h, w] = size(iR);
[hk, wk] = size(bk);

scale1 = h+hk-1;
scale2 = w+wk-1;

iR_pad = zeros(scale1,scale2);
iG_pad = zeros(scale1,scale2);
iB_pad = zeros(scale1,scale2);
bks_pad = zeros(scale1,scale2);

iR_pad(1:h,1:w) = iR;
iG_pad(1:h,1:w) = iG;
iB_pad(1:h,1:w) = iB;

bks_pad(1:hk,1:wk) = bk;

fftR= fftshift(fft2(iR_pad));
fftG= fftshift(fft2(iG_pad));
fftB= fftshift(fft2(iB_pad));

FK = fftshift(fft2(bks_pad));

for p=1:scale1
    for q=1:scale2  
        if abs(FK(p,q))<1000
            FK(p,q)=1000;
        end
    end
end

knew=10000000*get(hObject,'Value');

%knew= 200;
FK1 = ((FK.*conj(FK)));
FK2=((FK.*conj(FK))+knew); 
FK3 = FK2.*FK;
FK4 = FK1./FK3;


RB = fftR .* FK4;
GB = fftG .* FK4;
BB = fftB .* FK4;


iRB=ifft2(ifftshift((RB)));
iGB=ifft2(ifftshift((GB)));
iBB=ifft2(ifftshift((BB)));

bR = zeros(h,w);
bR= iRB(1:h,1:w);

bG = zeros(h,w);
bG= iGB(1:h,1:w);

bB = zeros(h,w);
bB= iBB(1:h,1:w);

finalblur_RGB= cat(3,bR,bG,bB);
finalblur_RGB=abs((255/max(finalblur_RGB(:))).*finalblur_RGB);

bf=real(finalblur_RGB)*2;

D=abs((255/max(bf(:))).*bf)*2;

img=bf;
img=double(img(:));
ima=max(img(:));
imi=min(img(:));
mse=std(img(:));
kout = (10*log10((ima^2)./mse));

txt2=num2str(kout);
set(handles.edit3,'String',txt2);

%[s,v]=ssim(bf,blur);

X = bf;
%figure, imshow(X);
X=rgb2gray(X);
%figure, imshow(X);
Y =blur;
%figure, imshow(Y);
Y=rgb2gray(Y);
%figure, imshow(Y);
Ux = mean(mean(X));
Uy = mean(mean(Y));
sigma_X = mean(mean((X-Ux).^2));
sigma_Y = mean(mean((Y-Uy).^2));
sigma_XY = mean(mean((X-Ux).*(Y-Uy)));
k1 = 0.01;
k2 = 0.03;
l =255;
c1 =k1*l;
c2 = k2*l;
ans= 0.40*( (2*Ux*Uy+c1)*(2*sigma_XY+c2) )/( (Ux^2+Uy^2+c1)*(sigma_X+sigma_Y+c2) );

txt3=num2str(ans);
set(handles.edit4,'String',txt3);

 axes(handles.axes3);
        imshow(uint8(bf));



% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider3_Callback(hObject,eventdata, handles);
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
%CLS
global im3 blur im2
 im3 = blur;
 i= im3;
 bk = im2;

iR=i(:,:,1);
iG=i(:,:,2);
iB=i(:,:,3);

[h, w] = size(iR);
[hk, wk] = size(bk);

scale1 = h+hk-1;
scale2 = w+wk-1;

iR_pad = zeros(scale1,scale2);
iG_pad = zeros(scale1,scale2);
iB_pad = zeros(scale1,scale2);
bks_pad = zeros(scale1,scale2);

iR_pad(1:h,1:w) = iR;
iG_pad(1:h,1:w) = iG;
iB_pad(1:h,1:w) = iB;

bks_pad(1:hk,1:wk) = bk;

fftR= fftshift(fft2(iR_pad));
fftG= fftshift(fft2(iG_pad));
fftB= fftshift(fft2(iB_pad));

FK = fftshift(fft2(bks_pad));

for p=1:scale1
    for q=1:scale2  
        if abs(FK(p,q))<1000
            FK(p,q)=1000;
        end
    end
end


P=[0,-1,0;-1,4,-1;0,-1,0];


lambda=1000000*get(hObject,'Value');

FK1 = ((FK.*conj(FK)));
FK2=((FK.*conj(FK))+lambda*norm(P)); 
FK3 = FK2.*FK;
FK4 = FK1./FK3;
% FK1 = (abs(FK).^2) ./(abs(FK).^2+k); 
% FK2 = FK1./FK;
% 
% 

RB = fftR .* FK4;
GB = fftG .* FK4;
BB = fftB .* FK4;


iRB=ifft2(ifftshift(RB));
iGB=ifft2(ifftshift(GB));
iBB=ifft2(ifftshift(BB));

bR = zeros(h,w);
bR= iRB(1:h,1:w);

bG = zeros(h,w);
bG= iGB(1:h,1:w);

bB = zeros(h,w);
bB= iBB(1:h,1:w);

finaldeblur_RGB= cat(3,bR,bG,bB);
finaldeblur_RGB=abs((255/max(finaldeblur_RGB(:))).*finaldeblur_RGB);

bf=real(finaldeblur_RGB)*2;

D=abs((255/max(bf(:))).*bf)*2;
img=bf;
img=double(img(:));
ima=max(img(:));
imi=min(img(:));
mse=std(img(:));
kout = (10*log10((ima^2)./mse));

txt2=num2str(kout);
set(handles.edit5,'String',txt2);

%[s,v]=ssim(bf,blur);

X = bf;
%figure, imshow(X);
X=rgb2gray(X);
%figure, imshow(X);
Y =blur;
%figure, imshow(Y);
Y=rgb2gray(Y);
%figure, imshow(Y);
Ux = mean(mean(X));
Uy = mean(mean(Y));
sigma_X = mean(mean((X-Ux).^2));
sigma_Y = mean(mean((Y-Uy).^2));
sigma_XY = mean(mean((X-Ux).*(Y-Uy)));
k1 = 0.01;
k2 = 0.03;
l =255;
c1 =k1*l;
c2 = k2*l;
ans= 0.40*( (2*Ux*Uy+c1)*(2*sigma_XY+c2) )/( (Ux^2+Uy^2+c1)*(sigma_X+sigma_Y+c2) );


txt3=num2str(ans);
set(handles.edit6,'String',txt3);
axes(handles.axes3);
imshow(uint8(bf));


% --- Executes during object creation, after setting all properties.
function slider3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
