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
from reportlab.lib.units import mm, inch
from reportlab.lib.colors import black, white
from reportlab.pdfgen import canvas
from StringIO import StringIO

class calTarget:
  fontName = 'Helvetica'
  fontSize = 12
  margins = (0.5*inch, 0.5*inch)
  
  def __init__(self, gridCount, gridSpacing=None):
    self.gridCount = gridCount;
    gridSpacing = 25 if gridSpacing is None else gridSpacing
    self.gridSpacing = gridSpacing*mm if hasattr(gridSpacing, "__len__") else (gridSpacing*mm, gridSpacing*mm)
    
    self.buffer = StringIO()
    self.canvas = canvas.Canvas(self.buffer, self.get_pageSize())
    self.canvas.setPageSize((1800,1250))
    self.draw()

  def save(self, filename):
    self.canvas.showPage()
    self.canvas.save()
    
    pdf = self.buffer.getvalue()
    self.buffer.close()
    
    with file(filename, 'wb') as f:
      f.write(pdf)

  def addTextFooter(self, str=None):
    if str is None:
      str = self.description()
  
    self.canvas.setFont(self.fontName, self.fontSize)
    self.canvas.setFillColorRGB(0.6,0.6,0.6)
    self.canvas.drawString(self.margins[0], self.margins[1]/2, str)

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
    self.canvas.setFillColor(black)
    for c in range(0, self.gridCount[0]):
      for r in range(0, self.gridCount[1]):
        if ( (r+c)%2 == 0):
          xy=(off[0]+c*self.gridSpacing[0], off[1]+r*self.gridSpacing[1])
          self.canvas.rect(xy[0], xy[1], self.gridSpacing[0], self.gridSpacing[1],
                           stroke=0, fill=1)

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
    self.canvas.setFillColor(black)
    self.canvas.circle(off[0], off[1], self.dotSize, stroke=1, fill=1)  # big corner dot
    for c in range(0, self.gridCount[0]):
      for r in range(0, self.gridCount[1]):
        xy=(off[0]+c*self.gridSpacing[0], off[1]+r*self.gridSpacing[1])
        self.canvas.setFillColor(black)
        self.canvas.circle(xy[0], xy[1], self.dotSize/2,stroke=0, fill=1)
        self.canvas.setFillColor(white)
        self.canvas.circle(xy[0], xy[1], .02*self.dotSize, stroke=0, fill=1)

def parse_args():
  parser = argparse.ArgumentParser(description='Generate calibration target')
  parser.add_argument('type', type=str, choices=['checker','dot'], help='Type of target')
  parser.add_argument('grid_X', type=int, help='# of grid points in X-direction')
  parser.add_argument('grid_Y', type=int, help='# of grid points in Y-direction')
  parser.add_argument('--spacing', type=float, default=25, help='Grid Spacing (mm)')
  parser.add_argument('--dotSize', type=float, default=10, help='Dot Size (mm)')
  parser.add_argument('--addFooter', type=int, default=1, help='add/remove footer')
  parser.add_argument('-o',dest='filename',type=str, default='calibrationTarget.pdf', help='output filename')
  return parser.parse_args()

if __name__=="__main__":
  args = parse_args()
  if (args.type == 'checker'):
    target = checkerTarget( (args.grid_X, args.grid_Y), gridSpacing=args.spacing)
  elif (args.type == 'dot'):
    target = dotTarget( (args.grid_X, args.grid_Y), gridSpacing=args.spacing, dotSize=args.dotSize)
    
  if(args.addFooter == 1):
    target.addTextFooter()
  target.save(args.filename)
