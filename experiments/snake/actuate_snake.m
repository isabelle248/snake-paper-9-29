function tangential_wave = actuate_snake(n_bends, t, amplitude, frequency, spatial_wavelength, phase_offset, loco_type)
% Target bend (natural curvature) of each joint at time t.
%   amplitude          : peak target bend of one joint, 2*tan(joint angle/2) (no unit)
%   frequency          : Hz
%   spatial_wavelength : wavelength in BODY LENGTHS
%   phase_offset       : constant phase (rad); it only shifts the time origin
%   loco_type          : 1 = land gait (uniform amplitude), 2 = aquatic gait (amplitude grows to the tail)

    % Position of each joint along the body, as a fraction of the body length.
    % Joint j sits at node j+1, which is j segments from the head, so s = j/(n_bends+1).
    % (The old code used linspace(0,1,n_bends), which measured the wavelength in units of
    %  the distance between the first and last joint = 0.9 body lengths.)
    s = (1:n_bends)/(n_bends + 1);

    omega = 2*pi*frequency;
    k = 2*pi/spatial_wavelength;

    switch loco_type
        case 1  % Land locomotion (uniform wave)
            tangential_wave = amplitude * sin(omega*t - k*s + phase_offset);

        case 2  % Aquatic locomotion (amplitude grows linearly toward the tail)
            A_s = amplitude * ((1 + 0.25*s)/1.25);
            tangential_wave = A_s .* sin(omega*t - k*s + phase_offset);

        otherwise
            error('Invalid locomotion strategy.');
    end
end
