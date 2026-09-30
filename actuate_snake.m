function tangential_wave = actuate_snake(n_bends, t, amplitude, frequency, spatial_wavelength, phase_offset, loco_type)
    % % --- Parameters you can tune ---
    % amplitude = 0.05;             % Max strain applied to each bend
    % frequency = 1.0;              % Oscillation frequency in Hz
    % spatial_wavelength = 2;     % Wavelength along the body in normalized units
    % phase_offset = pi / 2;        % Phase offset between vertical and tangential components

    % --- Compute strain pattern ---
    s = linspace(0, 1, n_bends);    % Normalized position along body

    % Traveling sinusoidal waves
    omega = 2 * pi * frequency;
    k = 2 * pi / spatial_wavelength;

    % 
    % % AQUATIC
    % % Make amplitude increase posteriorly
    % A_s = amplitude * (1 + 0.5*s);   % s=0->1, amplitude grows along body
    % 
    % % Phase offset increases posteriorly
    % phase_offset = pi /4 * s;  % Phase offset grows linearly along body
    % 
    % tangential_wave = A_s .* sin(omega * t - k * s + phase_offset);


    % % LAND
    % tangential_wave = amplitude * sin(omega * t - k * s + phase_offset);


    switch loco_type
    
        case 1  % Land locomotion (uniform wave)
            tangential_wave = amplitude * ...
                sin(omega*t - k*s + phase_offset);

        case 2  % Aquatic locomotion (posterior amplitude + posterior phase lead)
            % A_s = amplitude * (1 + 0.25*s);
            % phase_s = phase_offset * 0.5 * s;
            % tangential_wave = A_s .* ...
            %     sin(omega*t - k*s + phase_s);
            A_s = amplitude * ((1 + 0.25*s)/1.25);
            tangential_wave = A_s .* ...
                sin(omega*t - k*s + phase_offset);

        otherwise
            error('Invalid locomotion strategy.');
    end


end
