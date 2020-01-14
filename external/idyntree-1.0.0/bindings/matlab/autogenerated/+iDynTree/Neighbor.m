classdef Neighbor < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = neighborLink(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1146, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1147, self, varargin{1});
      end
    end
    function varargout = neighborJoint(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1148, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1149, self, varargin{1});
      end
    end
    function self = Neighbor(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1150, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1151, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
