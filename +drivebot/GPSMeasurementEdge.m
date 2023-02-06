classdef GPSMeasurementEdge < g2o.core.BaseUnaryEdge
    
    methods(Access = public)
        
        function this = GPSMeasurementEdge()
            this = this@g2o.core.BaseUnaryEdge(2);
        end
        
        function computeError(this)
            x = this.edgeVertices{1}.estimate();
            this.errorZ = x(1:2) - this.z;
%             warning('gpsmeasurementedge:computeerror:unimplemented', ...
%                 'Implement the rest of this method for Q1c.');
        end
        
        function linearizeOplus(this)
            this.J{1} = ...
                [1 0 0;
                0 1 0];
%             warning('gpsmeasurementedge:linearizeplus:unimplemented', ...
%                 'Implement the rest of this method for Q1c.');
        end
    end
end