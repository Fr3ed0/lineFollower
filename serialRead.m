function res = serialRead()
    % Define serial input at usb port COM4
    s = serial('COM4');

    % Define the serial refresh rate and timeout limit
    set(s, 'BaudRate', 250000, 'Timeout', .005);

    % Open the serial port for reading
    fopen(s);

    % Make a cleanup function to clear and close the serial port
    function cleanup(s)
        fclose(s); % close serial port
        delete(s); % delete port information
        clear s % remove port data from workspace
        disp(['Cleaned Up. Last data: ' num2str(serialData)]);
    end

    % Call the cleanup function when code is finished executing
    finishUp = onCleanup(@() cleanup(s));
    
    % Define looping variables
    receivingData = true;
    i = 0;

    while(receivingData)
        % If data is available through serial
        if(get(s, 'BytesAvailable') >= 1)
            % Add one to the index
            i = i + 1;

            % Retrieve a line of data from serial
            serialData = fscanf(s);

            % Scan the string received through serial for our three
            % important variables (distance, phi, dir)
            data = sscanf(serialData, '%f, %d, %d');

            % If the data receive through serial is empty, then stop serial
            % communication
            if (isempty(data))
                receivingData = false;
                disp('Serial collection terminated.');
            else
                % Store the data in the appropriate vector
                leftIR(i) = data(1);
                rightIR(i) = data(2);
                leftMotor(i) = data(3);
                rightMotor(i) = data(4);
            end
        % If no data is available, state the index
        else
            disp(['No current incoming data at index ' num2str(i)]);
        end
    end
    
    % Plot the collected data
    hold on
    plot(leftIR, 'r*')
    plot(rightIR, 'b*')
    plot(leftMotor, 'r')
    plot(rightMotor, 'b')
end