function LIDAr
    global fid
    fid = serial('COM4','BaudRate',115200);
    set(fid,'InputBufferSize',4000)  %  Set serial buffer size >(6+360x4)*2=2892 byte
    set(fid,'Timeout',1)             %  Set read Timeout in 1 sec
    fopen(fid);
    set(gcf, 'renderer', 'painters')
    
    finishup = onCleanup(@() closePorts(fid));
        
    visualisation = 1;

    offset = 140;
    init_level = 0;
    global index
    index = 0;
    
    global lidarData
    lidarData = zeros(2,360,'uint16');
    global l_dist
    l_dist = zeros(1, 360);

    l_qual = zeros(1, 360);
    
    
    figure;
    global lidarplot
    %lidarplot=plot([0 1],[1 0],'.'),% plot something
    global theta
    theta = linspace(0, 2*pi, 360);
    axis equal,axis([-500 500 -500 500]),grid on
    xlabel('X mm'),ylabel('Y mm'), hold on
    drawnow
    
    
    read_lidar();
end

function closePorts(port)
    fclose(port);
    disp('Ports Closed');
end

function update_view(angle,data)
    global l_dist
    global lidarplot
    global theta
    global lidarData
    
    x = data(1);
    x1= data(2);
    x2= data(3);
    x3= data(4);

    dist = bitor( x, bitshift(  bitand( x1, hex2dec('3f') ), 8) );
    qual = bitor( x2, bitshift( x3, 8 ) );
    %lidarData(:,angle) = [ dist, qual ];
    l_dist(angle) = dist;
    %disp(l_dist(angle))
    l_qual(angle) = qual;
    
    
    %dist=bitand(lidarData(1,:),uint16(hex2dec('3fff')*ones(1,360)));
    
    %[X,Y] = pol2cart(0:2*pi/360:2*pi*(1-1/360),double(dist));
    [X,Y] = pol2cart(theta, l_dist);


    cla
    hold on
    plot(X,Y,'.')
    % axis equal,axis([-500 500 -500 500]),grid on
    drawnow
    
    %pause(.01)
    
end

function r = compute_speed(data)
    speed_rpm = bitor(data(1), bitshift(data(2), 8)) / 64;
    r = speed_rpm;
end

function read_lidar()
    global lidarData
    global fid
    global lidarplot
    nb_errors = 0;
    loop = 1;
    index=0;
    XV11_Status=0; % 0~3 search for header, 4 has header, 5 timout/error.
    
    data=zeros(2,360,'uint16');

    while(XV11_Status<=3)
        %pause(0.00001)
    
        switch XV11_Status
            case 0 % no header
                % start byte
                b = fread(fid, 1, 'uint8');
                if(b==hex2dec('FA'))
                    XV11_Status=1;
                end
            case 1
                % position index
                b = fread(fid, 1, 'uint8');
                if(b>=hex2dec('A0') && b<=hex2dec('F9'))
                    index = b - hex2dec('A0');
                    XV11_Status=2;
                elseif(b~=hex2dec('FA'))
                    XV11_Status=0;
                end
            case 2
                % speed
                b_speed = fread(fid, 2, 'uint8');

                % data
                b_data0 = fread(fid, 4, 'uint8');
                b_data1 = fread(fid, 4, 'uint8');
                b_data2 = fread(fid, 4, 'uint8');
                b_data3 = fread(fid, 4, 'uint8');

                % for the checksum, we need all the data of the packet...
                % this could be collected in a more elegent fashion...
                %all_data = [hex2dec('FA'), index+hex2dec('A0')];

                speed_rpm = compute_speed(b_speed);

                update_view(index * 4 + 1, b_data0);
                update_view(index * 4 + 2, b_data1);
                update_view(index * 4 + 3, b_data2);
                update_view(index * 4 + 4, b_data3);


                XV11_Status=0;
                
            case 3
                fclose(fid);


        end

    end

end


    