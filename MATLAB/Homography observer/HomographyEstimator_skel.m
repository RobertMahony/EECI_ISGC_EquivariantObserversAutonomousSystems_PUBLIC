% Author: Minh-Duc HUA 05/08/2013
% HOMOGRAPHY ESTIMATION REPLAY FROM CAM+IMU DATA 
% Camera AVT Stingray 125B (40Hz)
% IMU AVT Stingray 125B (200Hz)

clc; clear all; close all; warning off;

% show and print image with 1
IMAGE_SHOW = 1;
IMAGE_PRINT = 0;

% image selection
FirstImage  = 0;
IMG_nbr     = 250;

% reference rectangle in image 0 
P1ref = [0;0]; P2ref = [800;0]; P3ref = [800;600]; P4ref = [0;600];

% initialize H and Gamma estimates 
Hhat = eye(3);
Hhat = Hhat/det(Hhat)^(1/3);
Gamma = [0 0 0; 0 0 0; 0 0 0];  

% time step
dt=0.005;

% Camera's intrinsic paramaters
Kcal = [448.85, 0, 394.306; 0, 450.264, 292.824; 0, 0, 1];
KcalInv = inv(Kcal);
% load IMU data
[cnt,gyr_x,gyr_y,gyr_z,acc_x,acc_y,acc_z,mag_x,mag_y,mag_z,nu1,nu2,nu3]=...
    textread('DATASET/measurements.txt','','delimiter',';','emptyvalue',NaN);
N = length(gyr_x);
gyr_data_x = zeros(N,1); gyr_data_y = zeros(N,1); gyr_data_z = zeros(N,1);
gain_gyr = 1.0/4474;
for i=1:N
  gyr_data_x(i) = ((gyr_x(i) - 32768) * gain_gyr) + 0.100368542288248;
  gyr_data_y(i) = ((gyr_y(i) - 32768) * gain_gyr) - 0.038747190889246;
  gyr_data_z(i) = ((gyr_z(i) - 32768) * gain_gyr) + 0.019727567928096;
end

% load 4 ESM-based conners data
[P1_x,P1_y,P2_x,P2_y,P3_x,P3_y,P4_x,P4_y]=...
    textread('DATASET/uv_esm.txt','%d%d%d%d%d%d%d%d');

% load reference image
referenceImage = 'DATASET/reference.png';
I_ref = imread(referenceImage); 
size_x_ref = size(I_ref,2);
size_y_ref = size(I_ref,1);

p1ref=Kcal\[P1ref(1); P1ref(2); 1];  p1ref=p1ref/sqrt(p1ref'*p1ref);
p2ref=Kcal\[P2ref(1); P2ref(2); 1];  p2ref=p2ref/sqrt(p2ref'*p2ref);
p3ref=Kcal\[P3ref(1); P3ref(2); 1];  p3ref=p3ref/sqrt(p3ref'*p3ref);
p4ref=Kcal\[P4ref(1); P4ref(2); 1];  p4ref=p4ref/sqrt(p4ref'*p4ref);
pref=[p1ref p2ref p3ref p4ref];

%% Homography estimation:
nbrPoints=4;
times = 1:IMG_nbr;
Lya = zeros(1,IMG_nbr);
PointsValid=zeros(1,nbrPoints);
Xmax=800; Ymax=600; 
time = 0:5*dt:(IMG_nbr-1)*5*dt;  

for i=FirstImage:(FirstImage+IMG_nbr-1)
    % check whether Points are inside the current image or not
    NbrPointObs = 0;
    PointsValid = PointsValid*0;
  
    if ((P1_x(i+1)*(P1_x(i+1)-Xmax)<0) && (P1_y(i+1)*(P1_y(i+1)-Ymax)<0))
        PointsValid(1)=1;  
        NbrPointObs = NbrPointObs + 1;
    end
    if ((P2_x(i+1)*(P2_x(i+1)-Xmax)<0) && (P2_y(i+1)*(P2_y(i+1)-Ymax)<0))
        PointsValid(2)=1;
        NbrPointObs = NbrPointObs + 1;
    end
    if ((P3_x(i+1)*(P3_x(i+1)-Xmax)<0) && (P3_y(i+1)*(P3_y(i+1)-Ymax)<0))
        PointsValid(3)=1;
        NbrPointObs = NbrPointObs + 1;
    end
    if ((P4_x(i+1)*(P4_x(i+1)-Xmax)<0) && (P4_y(i+1)*(P4_y(i+1)-Ymax)<0))
        PointsValid(4)=1;
        NbrPointObs = NbrPointObs + 1;
    end 
    
    % compute current points
    p1cur=Kcal\[P1_x(i+1);P1_y(i+1);1]; p1cur=p1cur/sqrt(p1cur'*p1cur);
    p2cur=Kcal\[P2_x(i+1);P2_y(i+1);1]; p2cur=p2cur/sqrt(p2cur'*p2cur);
    p3cur=Kcal\[P3_x(i+1);P3_y(i+1);1]; p3cur=p3cur/sqrt(p3cur'*p3cur);
    p4cur=Kcal\[P4_x(i+1);P4_y(i+1);1]; p4cur=p4cur/sqrt(p4cur'*p4cur);
    pcur=[p1cur p2cur p3cur p4cur];
  
%%
    % IMU is 5-times faster than Camera
    % calibration matrix Cam->IMU: 
    % = [0, 0, 1, 0.08; -1, 0, 0, 0.005; 0, -1, 0, 0.03;0 0 0 1]
    % Therefore, Rot(IMU->CAM) = [0 -1 0; 0 0 -1; 1 0 0]
  
    % Prediction Step
    for k=1:5
        ratex=-gyr_data_y((i)*5+k);
        ratey=-gyr_data_z((i)*5+k);
        ratez= gyr_data_x((i)*5+k);
        Omega_x = [   0       -ratez   ratey;... 
                      ratez    0      -ratex;...
                     -ratey    ratex   0    ];       
%%%%%%%%%%%%%% % COMPLETE START: add the part of the observer dynamics
        % that does NOT depend on the image point measurements
        % Omega_x: angular velocity in so(3)
        
        Hhat   = ? 
        Gamma =?
%%%%%%%%%%%%%%% COMPLETE END
    end
    
%% Observer gains
    if NbrPointObs >= 4
        kP=240; kI=1/16;
    else
        kP=120; kI=0; 
    end
%%
    % Correction step of Observer
    % COMPLETE START: add the innovation terms in the observer
    % kP: gain for Hhat
    % kI: gain for Gamma
    % pref: 3 x nbrPoints matrix of _normalized_ homogeneous coordinates of
    %       reference points, pref(:,j) is the j-th point
    % pcur: 3 x nbrPoints matrix of _normalized_ homogeneous coordinates of
    %       points in current image, pcur(:,j) is the j-th point
    % PointsValid(j): ==1 if jth point can be used, 0 otherwise
  
    
 
    
    
    
    Hhat   = ;
    Gamma  = ;   
    % COMPLETE END    
    
    %% Visualisation
    if IMAGE_SHOW==1
        hFig=figure(1);
        set(hFig,'Position', [10 50 800 480]);
                 
        s=subplot(2,2,1);
        set(s,'Position', [0.04 0.54 0.45 0.4]);
        currentImage = sprintf('DATASET/image_c0_%06.0f.PGM',i);
        I_cur = double(imread(currentImage));        
        imdisp(uint8(I_cur),'Border','tight');
        title(sprintf('Current Image %06.0f',i-FirstImage+1),'FontWeight','bold'); 
        hold on;
        plot([P1_x(i+1) P2_x(i+1) P3_x(i+1) P4_x(i+1) P1_x(i+1)],...
             [P1_y(i+1) P2_y(i+1) P3_y(i+1) P4_y(i+1) P1_y(i+1)],...
             '-rs','LineWidth',1,'MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',5);
         
        % transformed reference rectangle for visualisation
        HhatImginv = Kcal*(Hhat\KcalInv); % means HhatImginv = Kcal*inv(Hhat)*KcalInv;
        P1hatn = HhatImginv*[P1ref(1);P1ref(2);1]; P1hatn = P1hatn/P1hatn(3);
        P2hatn = HhatImginv*[P2ref(1);P2ref(2);1]; P2hatn = P2hatn/P2hatn(3);
        P3hatn = HhatImginv*[P3ref(1);P3ref(2);1]; P3hatn = P3hatn/P3hatn(3);
        P4hatn = HhatImginv*[P4ref(1);P4ref(2);1]; P4hatn = P4hatn/P4hatn(3); 
        plot([P1hatn(1) P2hatn(1) P3hatn(1) P4hatn(1) P1hatn(1)],...
             [P1hatn(2) P2hatn(2) P3hatn(2) P4hatn(2) P1hatn(2)],...
             '-gs','LineWidth',2,'MarkerEdgeColor','g','MarkerFaceColor','g','MarkerSize',2);
        
          
        % compute normalized current image
        I_curn = I_cur(:,:,1);
        cmin = min(I_curn(:));
        cmax = max(I_curn(:));    
        I_curn = I_curn - cmin;
        I_curn = I_curn ./ (cmax-cmin);

%% POUR TAREK, choisir isWarping = 1 pour afficher l'image warpée
        isWarping = 1;
        if isWarping == 1
            % re-compute transformed normalized current image
            I_transnc = homogWarp(I_curn,HhatImginv,size(I_curn),0);

            s=subplot(2,2,3);
            set(s,'Position', [0.04 0.04 0.45 0.4]);
            imdisp(I_transnc,'Border','tight');
            title('Warped Current Image','FontWeight','bold');

            % show reference image        
            s = subplot(2,2,4);
            set(s,'Position', [0.54 0.04 0.4 0.4]);  
            imdisp(I_ref,'Border','tight');
            text(P1ref(1),P1ref(2),'1','EdgeColor','blue','BackgroundColor','red');
            text(P2ref(1),P2ref(2),'2','EdgeColor','blue','BackgroundColor','red');
            text(P3ref(1),P3ref(2),'3','EdgeColor','blue','BackgroundColor','red');
            text(P4ref(1),P4ref(2),'4','EdgeColor','blue','BackgroundColor','red'); 
            title('Reference Image','FontWeight','bold');
        end
%% FIN DE COMMENTAIRE POUR TAREK

        % show Lyamunov function for verification
        Lya(i-FirstImage+1) = 0;
        for j=1:nbrPoints
            Lya(i-FirstImage+1) = Lya(i-FirstImage+1)+0.5*(e(:,j)-pref(:,j))'*(e(:,j)-pref(:,j));
        end        
        s = subplot(2,2,2);
        set(s,'Position', [0.54 0.54 0.4 0.4]);      
        plot(time(1:i-FirstImage+1),Lya(1:i-FirstImage+1),'r','LineWidth',2); grid on;
        axis([0 time(length(time)) 0 Lya(1)]);
        title('Measurement cost $\sum_i |e_i-p0_i|^2$','FontWeight','bold');
        
        drawnow;
        % save the figure
        if (IMAGE_PRINT==1)
            % Create a folder if it does not exist
            if ~exist('Results', 'dir')
                mkdir('Results');
            end
            filename = sprintf('Results/img%06.0f.png',i);
            print('-dpng',filename);     
        end
    end %if IMAGE_SHOW==1

end % for i=0:IMG_nbr 
 
