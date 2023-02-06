% The object state vertex:
% x(1) - x
% x(2) - y
% x(3) - z

classdef LandmarkStateVertex < g2o.core.BaseVertex
    
    properties(Access = protected)
        lId;
    end
    
    methods(Access = public)
        function this = LandmarkStateVertex(landmarkId)
            this=this@g2o.core.BaseVertex(2);
            this.lId = landmarkId;
        end
        
        function landmarkId = landmarkId(this)
            landmarkId = this.lId;
        end
    end
end