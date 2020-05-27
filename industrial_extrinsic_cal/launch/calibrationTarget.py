# This script provides functions to generate an image to be used as a target
# for calibrating cameras.  These functions can be incorporated into other
# scripts, or this program can be run directly (cmd line or GUI).
#
# Several options are supported, including:
#   - target style: checkerboard vs. dots
#   - grid dimensions (# of intersections/dots in Width/Height)
#   - grid spacing (between intersections/dots in Width/Height)
#   - dot size (for dot target)
#   - output format (png, pdf)
#   - DPI of images
#
# dependencies (install w/ "pip install ...")
#   - pillow (http://pypi.python.org/pypi/Pillow)
#   - reportlab (http://pypi.python.org/pypi/reportlab)

import argparse
import os.path
from reportlab.graphics.shapes import Drawing, Rect, Circle, String
from reportlab.graphics import renderPM, renderPDF
from reportlab.lib.units import mm, inch
from reportlab.lib.colors import Color, black, white

class calTarget:
  fontName = 'Helvetica'
  fontSize = 12
  margins = (0.5*inch, 0.5*inch)
  
  def __init__(self, gridCount, gridSpacing=None):
    self.gridCount = gridCount;
    gridSpacing = 25 if gridSpacing is None else gridSpacing
    self.gridSpacing = gridSpacing*mm if hasattr(gridSpacing, "__len__") else (gridSpacing*mm, gridSpacing*mm)
    
    ps = self.get_pageSize()
    self.drawing = Drawing(ps[0], ps[1])
    self.draw()

  def save(self, filename, dpi):
    ext = os.path.splitext(filename)[1][1:]
    if (ext == 'pdf'):
      renderPDF.drawToFile(self.drawing, filename)
    elif ext in ('png','jpg'):
      self.drawing.scale(dpi/72.0, dpi/72.0)
      renderPM.drawToFile(self.drawing, filename, dpi=dpi, fmt=ext)
    else:
      raise NotImplementedError('Unknown file type: '+ext)

  def addTextFooter(self, str=None):
    if str is None:
      str = self.description()
  
    self.drawing.add(String(self.margins[0], self.margins[1]/2, str,
                            fontName=self.fontName, fontSize=self.fontSize,
                            fillColor=Color(0.6,0.6,0.6)))

class checkerTarget(calTarget):
  def get_pageSize(self):
    return (self.gridCount[0]*self.gridSpacing[0]+self.margins[0]*2,
            self.gridCount[1]*self.gridSpacing[1]+self.margins[1]*2)

  def description(self):
    fmtStr = 'Checkboard Target, {0}x{1} grid, {2}x{3}mm spacing'
    str = fmtStr.format(self.gridCount[0], self.gridCount[1], self.gridSpacing[0]/mm, self.gridSpacing[1]/mm)
    return str

  def draw(self):
    off = self.margins
    for c in range(0, self.gridCount[0]):
      for r in range(0, self.gridCount[1]):
        if ( (r+c)%2 == 0):
          xy = (off[0]+c*self.gridSpacing[0], off[1]+r*self.gridSpacing[1])
          rect = Rect(xy[0], xy[1], self.gridSpacing[0], self.gridSpacing[1],
                      fillColor = black, strokeWidth = 0)
          self.drawing.add(rect)

class dotTarget(calTarget):
  def __init__(self, gridCount, gridSpacing=None, dotSize=10):
    self.dotSize = dotSize*mm
    calTarget.__init__(self, gridCount, gridSpacing)

  def get_pageSize(self):
    return ((self.gridCount[0]-1)*self.gridSpacing[0]+self.dotSize+self.margins[0]*3,
            (self.gridCount[1]-1)*self.gridSpacing[1]+self.dotSize+self.margins[1]*3)

  def description(self):
    fmtStr = 'Dot Target, {0}x{1} grid, {2}x{3}mm spacing, {4}mm dots'
    str = fmtStr.format(self.gridCount[0], self.gridCount[1], self.gridSpacing[0]/mm, self.gridSpacing[1]/mm, self.dotSize/mm)
    return str

  def draw(self):
    off = (self.margins[0]+self.dotSize, self.margins[1]+self.dotSize)
    self.drawing.add(Circle(off[0], off[1], self.dotSize, fillColor=black, strokeWidth=0))  # big corner dot
    for c in range(0, self.gridCount[0]):
      for r in range(0, self.gridCount[1]):
        xy=(off[0]+c*self.gridSpacing[0], off[1]+r*self.gridSpacing[1])
        self.drawing.add(Circle(xy[0], xy[1], self.dotSize/2, fillColor=black, strokeWidth=0))
        self.drawing.add(Circle(xy[0], xy[1], .03*self.dotSize, fillColor=white, strokeWidth=0))

def parse_args():
  parser = argparse.ArgumentParser(description='Generate calibration target')
  parser.add_argument('type', type=str, choices=['checker','dot'], help='Type of target')
  parser.add_argument('grid_X', type=int, help='# of grid points in X-direction')
  parser.add_argument('grid_Y', type=int, help='# of grid points in Y-direction')
  parser.add_argument('--spacing', type=float, default=25, help='Grid Spacing (mm)')
  parser.add_argument('--dotSize', type=float, default=10, help='Dot Size (mm)')
  parser.add_argument('--dpi', type=int, default=300, help='dots per inch (pixel formats only)')
  parser.add_argument('--addFooter', action='store_true', help='add footer text')
  parser.add_argument('-o',dest='filename',type=str, default='calibrationTarget.pdf', help='output filename (supports .pdf, .png, .jpg, ...)')
  return parser.parse_args()

if __name__=="__main__":
  args = parse_args()
  if (args.type == 'checker'):
    target = checkerTarget( (args.grid_X, args.grid_Y), gridSpacing=args.spacing)
  elif (args.type == 'dot'):
    target = dotTarget( (args.grid_X, args.grid_Y), gridSpacing=args.spacing, dotSize=args.dotSize)
    
  if(args.addFooter):
    target.addTextFooter()
  target.save(args.filename, args.dpi)
