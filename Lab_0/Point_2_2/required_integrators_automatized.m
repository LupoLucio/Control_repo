%% REQUIRED_INTEGRATORS_AUTOMATIZED Compute required integrators for steady-state error
%  Calculates the number of integrators needed in the controller to achieve
%  zero steady-state error for the specified input signal type.
%
%  Syntax: [number_of_integrators] = required_integrators_automatized(P, signal_type)
%
%  Inputs:
%    P            - Transfer function of the plant [tf object]
%    signal_type  - Type of reference signal: 'impulse', 'step', 'linear ramp', 'parabolic ramp'
%
%  Output:
%    number_of_integrators - Number of integrators required (0, 1, 2, or 3)
%
%  Example:
%    required = required_integrators_automatized(P, 'step');
%
function [number_of_integrators] = required_integrators_automatized(P,signal_type)

        % Constants
        POLE_ORIGIN_TOLERANCE = 1e-6;  % Tolerance for detecting poles at origin [rad/s]
        
        % Input validation
        if ~isa(P, 'tf') && ~isa(P, 'ss') && ~isa(P, 'zpk')
            error('required_integrators_automatized:InvalidInput', ...
                  'Plant P must be a transfer function (tf, ss, or zpk object)');
        end
        
        signal_types = {'impulse', 'step', 'linear ramp', 'parabolic ramp'};
        if ~ismember(signal_type, signal_types)
            error('required_integrators_automatized:InvalidSignal', ...
                  sprintf('Unknown signal type: %s. Valid options: %s', ...
                  signal_type, strjoin(signal_types, ', ')));
        end
        
        % Map signal type to required integrators
        switch signal_type
            case 'impulse'
                integrators = 0;
            case 'step'
                integrators = 1;
            case 'linear ramp'
                integrators = 2;
            case 'parabolic ramp'
                integrators = 3;
        end
        
        % Count poles at origin (within tolerance)
        all_poles = pole(P);
        poles_at_origin = sum(abs(all_poles) < POLE_ORIGIN_TOLERANCE);
        
        % Compute required additional integrators
        number_of_integrators = integrators - poles_at_origin;
        
        if number_of_integrators < 0
            warning('required_integrators_automatized:ExcessIntegrators', ...
                    'Plant already has more integrators than required. Result: %d', ...
                    number_of_integrators);
            number_of_integrators = 0; % Clamp to non-negative
        end

end
