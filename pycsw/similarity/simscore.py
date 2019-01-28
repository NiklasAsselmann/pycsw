import math
import heapq
import datetime
import dateutil.parser
from shapely.geometry import Polygon, mapping, shape
from pyproj import Proj



'''
@author: Carolin Wortmann



Help functions:

    checkValidity:      checks validity for getSimilarityScores inputs

    convertToRadians:   converts degree to radians

    getDiagonal:        gets diagonal length of bounding box (from entry as dict)

    getInterv:          gets length of temporal interval (from entry as dict)

    getCenter:          calculates center of bounding box

    getAr/getAres:      calculates area of bouunding box on earth surface

'''
'''Validitiy Check
Checks whether entries are valid
input:
    entries:    list of dicts, each dict containing an entry (see bottom for more info)
    ent:        single entry as dict
    n:          number of similar records to be shown
    ext:          weight of extent similarity 
    dat:          weight of datatype similarity
    loc:          weight of location similarity
    geo:          weight of spatial similarity
    tim:          weight of temporal similarity
    mxm:          max value for weights

output:
    true:       if entries and cmp are all valid entries, n is a natural number, e,d,l,g,t are all 
                greater or equal 0 and smaller or equal 5
    false:      else
    
'''
def checkValidity(entries, ent, n, ext, dat, loc, geo, tim, mxm, dtl):
    #entries will be checked during iteration in main function
    #ent
    if ent is None or ent["id"] is None:
        raise ValueError('no comparable input')

    #n will be checked inside main function

    #parameters

    if mxm<0 or ext<0 or ext>mxm or dat<0 or dat>mxm or loc<0 or loc>mxm or geo<0 or geo>mxm or tim<0 or tim>mxm or (geo==0 and tim==0):
        raise ValueError('invalid parameters, must be within 0 and mxm, geo and tim cannot both be 0')
    return 



def checkTempInput(entry):
    if entry["time"] is None or entry["time"][0] is None or entry["time"][0]==0 or entry["time"][1] is None or entry["time"][1]==0:
        return False
    return True 


def checkBboxInput(entry):
    if entry["wkt_geometry"] is None or len(entry["wkt_geometry"])<=3:
        return False
    return True

def checkVectorInput(entry):
    if entry["vector"] is None or len(entry["vector"])<=2:
        return False
    return True


'''
Converts value in degree to radians
    input:  value in degree
    output: input value converted to radians
'''
def ConvertToRadian(input):
    return float(input) * math.pi/ 180


#Calculates diagonal of Bounding Box by use of Haversine Formula


''' Calculation of diagonal length
    Calculates length of bounding box for entry
    input: 
        entry : record from repository
    output:   
        diagonal length in m
'''
def getDiagonal(entry):
    lon1 = entry["wkt_geometry"][1]
    lon2 = entry["wkt_geometry"][3]
    lat2 = entry["wkt_geometry"][2]
    lat1 = entry["wkt_geometry"][0]

    return gDiag(lat1,lat2,lon1,lon2)

''' Calculate Diagonal - inner function
Calculates length of line between two points
    input: 
        lat1, lon1 : coordinates first point
        lat2, lon2 : coordinates second point

    output: 
        diagonal length in m
'''
def gDiag(lat1,lat2,lon1,lon2):
    # convert decimal degrees to radians 
    lon1, lat1, lon2, lat2 = map(ConvertToRadian, [lon1, lat1, lon2, lat2])

    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a)) 
    r = 6378137 
    d = c * r
    if dlon == 0 and dlat == 0:
        return 0
    return d


'''Calculate duration
Calculates the length of temporal interval 
input:
    entry : time information from entry
output:
    timedelta
'''
def getInterv(entry):
    if entry is None or entry[0] is None or entry[1] is None:
        return 0
    t1 = entry[0]
    t2 = entry[1]
    frmt = "%Y-%m-%dT%H:%M:%SZ" 
    tdelta = datetime.datetime.strptime(t2, frmt) - datetime.datetime.strptime(t1, frmt)
    return tdelta

'''Calculate center of bbox
input:
    entry : repository record 
output:
    center of Bounding Box as 2D point
'''
def getCenter(entry):
    lon1=entry["wkt_geometry"][1]
    lon2=entry["wkt_geometry"][3]
    lat1=entry["wkt_geometry"][0]
    lat2=entry["wkt_geometry"][2]

    lon1, lat1, lon2, lat2 = map(ConvertToRadian, [lon1, lat1, lon2, lat2])

    dLon = lon2-lon1
    x = math.cos(lat2) * math.cos(dLon)
    y = math.cos(lat2) * math.sin(dLon)
    lat = math.atan2(math.sin(lat1)+math.sin(lat2),math.sqrt( (math.cos(lat1)+x)*(math.cos(lat1)+x) + y*y) )
    lon = lon1 + math.atan2(y, math.cos(lat1) + x)

    lon = (lon+540)%360-180

    center = [lon,lat]
    return center


'''

'''
def getPolygCenter(coord):
    #lon, lat = zip(*entry['vector'])
    
    #pa = Proj("+proj=aea +lat_1=37.0 +lat_2=41.0 +lat_0=39.0 +lon_0=-106.55")
    #equal area projection centered on and bracketing the area of interest
    #x, y = pa(lon, lat)
    #pol = {"type": "Polygon", "coordinates": [zip(x, y)]}
    pol=Polygon(coord)
    return list(pol.centroid.coords)




'''Calculate  - inner function
input:
    coordinates : list of coordinates
output:
    area size in m²
'''
def getArea(coordinates):
    area = 0

    if (len(coordinates)>2):
        i=0
        p1Lon = float(coordinates[i])
        p1Lat = float(coordinates[i+1])
        p2Lon = float(coordinates[i+2])
        p2Lat = float(coordinates[i+3])
        area += ConvertToRadian(p2Lon - p1Lon) * (2 + math.sin(ConvertToRadian(p1Lat)) + math.sin(ConvertToRadian(p2Lat)))

        area = area * 6378137 * 6378137 / 2
    
    return abs(area)


''' Calculate area
input: 
    points : list of coordinates
output:
    area of bounding box given by coordinates in m²
'''
def getAr(points):
    if (points[0]==points[1]) or (points[2]==points[3]):
        return 0
    return getArea(points)



'''
Polygon Area in m²
'''
def getPolAr(entry):
    lon, lat = zip(*entry['vector'])
    
    minLat=entry['wkt_geometry'][0]
    maxLat=entry['wkt_geometry'][2]
    minLon=entry['wkt_geometry'][1]
    maxLon=entry['wkt_geometry'][3]

    pa = Proj("+proj=aea +lat_1="+str(minLat)+" +lat_2="+str(maxLat)+" +lat_0="+str(minLon)+" +lon_0="+str(maxLon))
    #equal area projection centered on the area of interest
    x, y = pa(lon, lat)
    pol = {"type": "Polygon", "coordinates": [zip(x, y)]}
    return shape(pol).area 


def hasArea(entry):
    if getPolAr(entry)==0:
        print(str(getPolAr(entry)))
        return False
    return True


'''
Align Polygons by normalizing their sizes and moving polygonB on top of polygonA

'''


def uniformPolygonArea(entry):
    norm = 1000
    eArea = getPolAr(entry)
    fac = math.sqrt(norm/eArea)
    coords = entry["vector"]
    for i in coords:
        for j in i:
            j=j*fac
    #coords = map(lambda x: fac*x, entry["vector"])
    return coords




def moveCoordinates(coordsA, coordsB):
    ctrA=getPolygCenter(coordsA)
    ctrB=getPolygCenter(coordsB)
    diffX=ctrA[0][0]-ctrB[0][0]
    diffY=ctrA[0][1]-ctrB[0][1]
    for x in coordsB:
        x[0]=x[0]+diffX
        x[1]=x[1]+diffY
    return[coordsA,coordsB]


def getAlignedPolygons(entryA,entryB):
    unAreaA= uniformPolygonArea(entryA)
    unAreaB= uniformPolygonArea(entryB)
    return (moveCoordinates(unAreaA,unAreaB))





'''Points within limits of Bounding Box?
Checks whether corner points of rectangle B are in rectangle A
input: 
    pointsA : list of coordinate points for rectangle A
    pointsB : list of coordinate points for rectangle B
output:
    list of length 4, wherein each represents whether the corresponding point of B is in A (1) or not (0)
        The points correspond as follow:    index 0 : top left corner (MinLon,MaxLat)
                                            index 1 : top right corner (MaxLon,MaxLat)
                                            index 2 : bottom left corner (MinLon,MinLat)
                                            index 3 : bottom right corner (MaxLon,MinLat)
'''
def pointsInBbox(pointsA, pointsB):
    points = [[pointsB[1],pointsB[2]], [pointsB[1],pointsB[3]], [pointsB[0],pointsB[2]], [pointsB[0],pointsB[3]]]

    minLat=pointsA[0]
    maxLat=pointsA[2]
    minLon=pointsA[1]
    maxLon=pointsA[3]

    i=0
    res = [0,0,0,0]

    for x in points:
        if (minLon<=x[1] and x[1]<=maxLon and minLat<=x[0] and x[0]<=maxLat):
            res[i]=1
        i=i+1

    return res    


'''
Extent Similarity: 

    getGeoExtSim:       compares geographic extent by comparing diagonal lengths for two entries given as dicts

    getTempExtSim:      compares temporal extent by comparing temporal interval lengths for two entries given as dicts


'''


'''Similarity of geographical extent
Calculates similarity of geographical extent based on length of diagonals of bounding boxes
input:
    entryA, entryB : records from repository which are to be compared
output:
    similarityscore (in [0,1])

'''
def getGeoExtSim(entryA, entryB):
    diagonalA=float(getDiagonal(entryA))
    diagonalB=float(getDiagonal(entryB))
    minV = min(diagonalA, diagonalB)
    maxV = max(diagonalA, diagonalB)
    if maxV == 0:
        return 1
    sim = float(minV/maxV)
    return sim


'''Similarity of geographical extent
Calculates similarity of geographical extent based on length of diagonals of bounding boxes
input:
    entryA, entryB : records from repository which are to be compared
output:
    similarityscore (in [0,1])
TODO: implement more specific calc
'''
def getGeoExtSimE(entryA, entryB):
    areaA=getPolAr(entryA)
    areaB=getPolAr(entryB)
    minV = min(areaA, areaB)
    maxV = max(areaA, areaB)
    if maxV == 0:
        return 1
    sim = float(minV/maxV)
    return sim



'''Similarity of temporal extent
Calculates similarity of geographical extent based on length of diagonals of bounding boxes
input:
    entryA, entryB : records from repository which are to be compared
output:
    similarityscore (in [0,1])
'''
def getTempExtSim(entryA,entryB):
    extA = getInterv(entryA["time"]).total_seconds()
    extB = getInterv(entryB["time"]).total_seconds()
    if extA==0 or extB==0:
        return 0
    if extA==0:
        extA=1
    if extB==0:
        extB=1
    minV = min(extA, extB)
    maxV = max(extA, extB)
    sim = float(minV/maxV)
    return sim


'''
Location Similarity

    Intersection Similarity

        getInterGeoSim:         calculates ratio between intersection of both bounding boxes and first entry, 0 if disjunct, given two entries as dicts

        getInterTempSim:        calculates ratio between intersection of both entries on timeline and first entry, 0 is disjunct, given two entries as dicts

    Center Similarity:

        getCentGeoSim:          calculates difference between centers of bounding boxes of two entries, given as dicts, and calculates ratio to absolute maximum (half the earth's circumference)        

        getCentTempSim:         calculates difference between centers of temporal intervals of two entries, given as dicts, and calculates ratio to absolute max (to be determined)

'''

#####################################################################
####### Shape comparison ############################################
#####################################################################

def getShapeSim(entryA, entryB):
    #both are lines
    if not hasArea(entryA) and not hasArea(entryB):
        return 1
    #one line, one polygon
    if not hasArea(entryA) or not hasArea(entryB):
        print(str(entryB['id'])+" has no area")
        return 0
    polygs = getAlignedPolygons(entryA,entryB)
    polygonA = Polygon(polygs[0])
    polygonB = Polygon(polygs[1])
    intersec = (polygonA.intersection(polygonB)).area
    sim = intersec/(polygonA.area+(polygonB.area-intersec))
    return sim 


#####################################################################
####### Relation of absolute positions in coordinate systems ########
#####################################################################

'''Similarity of geographic location based on centers
Calulates similarity based on geographic location of centers of bounding boxes
input: 
    entryA, entryB : records from repository which are to be compared
output:
    similarityscore (in [0,1])

'''

def getCenterGeoSim(entryA, entryB):
    centerA = getCenter(entryA)
    centerB = getCenter(entryB)
    diagonal = gDiag(centerA[1], centerB[1], centerA[0], centerB[0])
    circumf = 20038000
    sim = 1-(diagonal/circumf)
    return sim


def getCenterTempSim(entryA, entryB):
    if getInterv(entryA["time"])==0 or getInterv(entryB["time"])==0:
        return 0

    frmt = "%Y-%m-%dT%H:%M:%SZ" 
    if entryA["time"][0]==entryA["time"][1]:
        centerA=entryA["time"][0]
    else: 
        centerA=datetime.datetime.strptime(entryA["time"][0],frmt)+(getInterv(entryA["time"])/2)
    if entryB["time"][0]==entryB["time"][1]:
        centerB=entryB["time"][0]
    else: 
        centerB=datetime.datetime.strptime(entryB["time"][0],frmt)+(getInterv(entryB["time"])/2)

    tdelta = centerA-centerB
    tdelta = tdelta.total_seconds()

    max = datetime.timedelta(days=365000).total_seconds()

    return tdelta/max


#########################################################################
########### Intersections ###############################################
#########################################################################


'''Calculate similarity based on intersecting area of bounding boxes
Calculates whether two records intersect and, if so, size of intersection area in relation to size of record A
input:
    entryA,entryB : records from repository which are to be compares
output: 
    similarityscore (in[0,1])
'''

# Calculate intersection area of both bounding boxes
def getInterGeoSim(entryA,entryB):
    minLatA=entryA["wkt_geometry"][0]
    maxLatA=entryA["wkt_geometry"][2]
    minLonA=entryA["wkt_geometry"][1]
    maxLonA=entryA["wkt_geometry"][3]
    minLatB=entryB["wkt_geometry"][0]
    maxLatB=entryB["wkt_geometry"][2]
    minLonB=entryB["wkt_geometry"][1]
    maxLonB=entryB["wkt_geometry"][3]
    

    #A in B 
    if minLonA >= minLonB and maxLonA <= maxLonB and minLonA >= minLonB and maxLonA <= maxLonB:
        return 1

    #disjunct?
    if minLonA > maxLonB or maxLonA < minLonB or maxLatA < minLatB or minLatA > maxLonB:
        return 0


    areaA = getAr(entryA["wkt_geometry"])

    #how many points of B in A?
    points = pointsInBbox(entryA["wkt_geometry"], entryB["wkt_geometry"])

    minLat=minLatA
    minLon=minLonA
    maxLat=maxLatA
    maxLon=maxLonA

    if points[0]==1:
        minLon=minLonB
        maxLat=maxLatB
    
    if points[1]==1:
        maxLat=maxLatB
        maxLon=maxLonB
    
    if points[2]==1:
        minLat=minLatB
        minLon=minLonB

    if points[3]==1:
        minLat=minLatB
        maxLon=maxLonB
    
    #If only points of B are in A, but not of A in B
    elif points[0]==0 and points[1]==0 and points[2]==0:
        points = pointsInBbox(entryB["wkt_geometry"], entryA["wkt_geometry"])
        unchanged = True
        
        if points[0]==1:
            maxLon=maxLonB
            minLat=minLatB
            unchanged = False
    
        if points[1]==1:
            minLat=minLatB
            minLon=minLonB
            unchanged = False
    
        if points[2]==1:
            maxLat=maxLatB
            maxLon=maxLonB
            unchanged = False

        if points[3]==1:
            maxLat=maxLatB
            minLon=minLonB
            unchanged = False
        
       # intersection when one rectangle has both min and max Latitude and the other has min and max Longitude
        if unchanged:
            if minLatA<minLatB and maxLatA>maxLatB:
                maxLat=maxLatB
                minLat=minLatB
                minLon=minLonA
                maxLon=maxLonA
            else:
                maxLat=maxLatA
                minLat=minLatA
                minLon=minLonB
                maxLon=maxLonB
    intersecarea=getAr([minLat,maxLat,minLon,maxLon])

    # if areaA is 0, A is either a point or a line. If it is a point, both cases (in B or outside B) are covered including return value above
    # if A is a line and it is not entirely in B, the insersection area must still be 0:
    if intersecarea == 0:
        # Wenn A eine Linie ist, länge der geschnittenen Linie mit Länge der Gesamtlinie vergleichen
        if areaA==0:
            line = gDiag(minLat,maxLat,minLon,maxLon)
            lineA = getDiagonal(entryA)
            return float (line/lineA) 
    return float(intersecarea/areaA)


'''Calculate similarity based on intersecting area of Polygon
Calculates whether two records intersect and, if so, size of intersection area in relation to size of record A
input:
    entryA,entryB : records from repository which are to be compares
output: 
    similarityscore (in[0,1]) 

TODO: implement exact calculation
'''

# Calculate intersection area of both bounding boxes 
def getInterGeoSimE(entryA,entryB):
    polygonA = Polygon(entryA['vector'])
    polygonB = Polygon(entryB['vector'])
    intersec = (polygonA.intersection(polygonB)).area
    sim = intersec/polygonA.area
    return sim 



''' Calculate similarity based on temporal intersection
Calculates whether two temporal intervals overlap, and if so, the relation between the overlap duration and the duration of record A
input:
    entryA,entryB : records from repository which are to be compared
output: 
    similarityscore (in[0,1])
'''
def getInterTempSim(entryA,entryB):
    frmt = "%Y-%m-%dT%H:%M:%SZ" 
    if getInterv(entryA["time"])==0 or getInterv(entryB["time"])==0:
        return 0
    interv=float(0)
    #starting points of intervals A, B
    startA = datetime.datetime.strptime(entryA["time"][0], frmt)
    endA = datetime.datetime.strptime(entryA["time"][1], frmt)
    startB = datetime.datetime.strptime(entryB["time"][0], frmt)
    endB = datetime.datetime.strptime(entryB["time"][1], frmt)

    lengthA=getInterv(entryA["time"]).total_seconds()

    #disjunct
    if startA>endB or startB>endA:
        return 0
        
    elif startA>startB:
        #A in B
        if endA<endB:
            return 1
        #intersection, B starts earlier than A
        else:
            interv = getInterv([startA,endB])
    #intersection, A starts earlier than B
    elif startB>startA:
        #B in A
        if endB<endA:
            interv = getInterv(entryB["time"]).total_seconds()
        else:
            interv = getInterv([startB,endA]).total_seconds()
    
    res = interv/lengthA
    return res


'''
Datatype Similarity 

    getGeoDatSim:       compares datatype of geographic information, given two entries as dicts

    getTempDatSim:      compares datatype of temporal information, given two entries as dicts


'''

'''Similarity calculation of datatype
Calulates similarity of geographic data types (is raster or vector? and if vector, same type of shape?)
input:
    entryA,entryB : records from repository which are to be compared
output: 
    similarityscore (in[0,1])
'''
def getGeoDatSim(entryA,entryB):
    if entryA["raster"]:
        if entryB["raster"]:
            return 1
        else:
            return 0
    if entryB["raster"]:
        return 0
    if len(entryA["vector"])>=3:
        if len(entryB["vector"])>=3:
            return 1
        else:
            return 0.8
    elif len(entryA["vector"])==len(entryB["vector"]):
        return 1
    else:
        return 0.8
    
'''Similarity of temporal data types
Calculates whether both records have same type of temporal data (interval or point)
input:
    entryA,entryB : records from repository which are to be compared
output: 
    similarityscore (in[0,1])

'''
def getTempDatSim(entryA,entryB):
    if entryA["time"][0]==0 or entryB["time"][0]==0:
        return 0
    if entryA["time"][0]==entryA["time"][1]:
        if entryB["time"][0]==entryB["time"][1]:
            return 1
        else:
            return 0
    elif entryB["time"][0]==entryB["time"][1]:
        return 0
    else:
        return 1



'''Comibination of individual similarity scores
combines Geo and Temp Similarites for selected criterium c while taking into consideration weights for geographic and temporal similarity
input:
    entryA,entryB : records from repository which are to be compared
    c : criterium 
        c=0 for Similarity of extent
        c=1 for Similarity of location
        c=2 for Similarity of datatype
    g : weight of geographic similarity
    t : weight of temporal similarity
output: 
    similarityscore (in[0,1])        
'''
def getIndSim(entryA, entryB, geo, tim, cri):
    tempA = checkTempInput(entryA)
    tempB = checkTempInput(entryB)
    bboxA = checkBboxInput(entryA)
    bboxB = checkBboxInput(entryB)
    vectorA = checkVectorInput(entryA)
    vectorB = checkVectorInput(entryB)

    geoSim = 0
    tempSim = 0

# Extent
    if cri==0:
        if bboxA and bboxB:
            geoSim = getGeoExtSim(entryA,entryB)
        if tempA and tempB:
            tempSim = getTempExtSim(entryA,entryB)
# Location    
    if cri==1:
        geoInter = 0
        tempInter = 0
        geoLoc = 0
        tempLoc = 0

        if bboxA and bboxB:
            geoInter = getInterGeoSim(entryA,entryB)
            geoLoc = getCenterGeoSim(entryA,entryB)
        if tempA and tempB:
            tempInter = getInterTempSim(entryA,entryB)
            tempLoc = getCenterTempSim(entryA,entryB)
        geoSim = 0.4*geoInter + 0.6*geoLoc
        tempSim = 0.4*tempInter + 0.6*tempLoc

# Datatype
    if cri==2:
        if vectorA and vectorB:
            geoSim = getGeoDatSim(entryA,entryB)
        if tempA and tempB:
            tempSim = getTempDatSim(entryA,entryB)

    rel = geo/(geo+tim)
    sim = rel*geoSim + (1-rel)*tempSim

    return sim 


'''Comibination of more exact calculation of individual similarity scores
combines Geo and Temp Similarites for selected criterium c while taking into consideration weights for geographic and temporal similarity
input:
    entryA,entryB : records from repository which are to be compared
    cri : criterium 
        c=0 for Similarity of extent
        c=1 for Similarity of location
    geo : weight of geographic similarity
    tim : weight of temporal similarity
output: 
    similarityscore (in[0,1])        
'''
def getExSim(entryA, entryB, geo, tim, cri):
    tempA = checkTempInput(entryA)
    tempB = checkTempInput(entryB)
    bboxA = checkBboxInput(entryA)
    bboxB = checkBboxInput(entryB)
    vectorA = checkVectorInput(entryA)
    vectorB = checkVectorInput(entryB)

    geoSim = 0
    tempSim = 0

# Extent
    if cri==0:
        if vectorB and vectorA:
            geoSim = getGeoExtSimE(entryA,entryB)
        elif bboxA and bboxB:
            geoSim = getGeoExtSim(entryA,entryB)
        if tempA and tempB:
            tempSim = getTempExtSim(entryA,entryB)
# Location    
    if cri==1:
        geoInter = 0
        tempInter = 0
        geoLoc = 0
        tempLoc = 0

        if vectorA and vectorB:
            geoInter = getInterGeoSimE(entryA,entryB)
            geoLoc = getCenterGeoSim(entryA,entryB)
        elif bboxA and bboxB:
            geoInter = getInterGeoSim(entryA,entryB)
            geoLoc = getCenterGeoSim(entryA,entryB)
        if tempA and tempB:
            tempInter = getInterTempSim(entryA,entryB)
            tempLoc = getCenterTempSim(entryA,entryB)
        geoSim = 0.4*geoInter + 0.6*geoLoc
        tempSim = 0.4*tempInter + 0.6*tempLoc

    # Shape
    if cri==3:
        geoSim=0
        if vectorA and vectorB:
            geoSim=getShapeSim(entryA, entryB)
        return geoSim

    rel = geo/(geo+tim)
    sim = rel*geoSim + (1-rel)*tempSim
    return sim 





def getSimScoreTotal(entryA, entryB, geo, tim, ext, dat, loc, mxm, dtl):

    sSim=None

    dSim = getIndSim(entryA, entryB, geo, tim, 2)
    if dtl is None or not dtl or not checkVectorInput(entryA) or not checkVectorInput(entryB): 
        lSim = getIndSim(entryA, entryB, geo, tim, 1)
    else: 
        lSim = getExSim(entryA, entryB, geo, tim, 1)
    if dtl is None or not dtl or not checkVectorInput(entryA) or not checkVectorInput(entryB): 
        eSim = getIndSim(entryA, entryB, geo, tim, 0)
    else: 
        eSim = getExSim(entryA, entryB, geo, tim, 0)
    if dtl and checkVectorInput(entryA) and checkVectorInput(entryB) and loc>0 and ext<0:
        sSim = getExSim(entryA, entryB, geo, tim, 3)
        totalSum=ext+dat+loc+((ext+loc)/2)
    else:
        totalSum=ext+dat+loc

    simScore=0

    if ext>0:
        simScore = simScore+(ext/totalSum*eSim)
    if loc>0:
        simScore = simScore+(loc/totalSum*lSim)
    if dat>0:
        simScore = simScore+(dat/totalSum*dSim)
    if sSim is not None:
        simScore = simScore+(((ext+loc)/2)/totalSum*sSim)

    simScore = 0.999*simScore
    
    return simScore


'''
getSimilarityScore: Berechnet den SimilarityScore
        entries: Expects a list of entries (dictionaries), where each dict represents one entry of the repository.

                entry:      {
                                "id" : idOfTheEntry,
                                "wkt_geometry" : [minLat, minLon, maxLat, maxLon],
                                "vector" : [[lat,long],[lat,long]...],
                                "time" : [start, end],
                                "raster"  : bool
                            }   

        ent is an entry and therefor the same format
        n : number of similar records to be retrieved
        ext : weight of extent similarity 
        dat : weight of datatype similarity 
        loc : weight of location similarity
        geo : weight geographic similarity
        tim : weight temporal similarity
        max : max value for weights
        dtl : boolean, true if detailed
'''

def getSimilarRecords(entries, ent, n, ext, dat, loc, geo, tim, mxm, dtl):
    
    checkValidity(entries, ent, n, ext, dat, loc, geo, tim, mxm, dtl)
    

    if n>len(entries)-1:
        n=len(entries)-1

    records = []

    i=0

    # First n entries are added to the priorityqueue
    while i < n and i<len(entries):
        if not (entries[i]["id"]==ent["id"]):
            heapq.heappush(records, [entries[i]["id"], getSimScoreTotal(ent, entries[i], geo, tim, ext, dat, loc, mxm, dtl)])
            i=i+1
        else:
            i=i+1
            n=n+1
    
    # Rest of entries are checked for better simscores
    while i < len(entries):
        min = heapq.heappop(records)
        currscore = getSimScoreTotal(ent, entries[i], geo, tim, ext, dat, loc, mxm, dtl)
        if min[1]<currscore and not (entries[i]["id"]==ent["id"]):
            heapq.heappush(records, [entries[i]["id"], getSimScoreTotal(ent, entries[i], geo, tim, ext, dat, loc, mxm, dtl)])
        else:
            heapq.heappush(records, min)
        i=i+1
    
    output=sorted(records, key= lambda x: x[1], reverse=True)

    return output


#####################################################################
################ Testdata ###########################################
#####################################################################

muenster1t1 ={
        "id": 'urn:uuid: 1a', 
        "wkt_geometry": [51.917591, 7.549667, 52.00137, 7.704849], 
        'time': ['2000-01-11T02:28:00Z', '2010-01-11T02:28:00Z'], 
        'vector': [[52.00137, 7.549667], [52.00137, 7.704849], [51.917591, 7.704849], [51.917591, 7.549667]], 
        'raster': False}

#################################
##### different temp extent #####
#################################

#same length, no intersection
muenster1t2 ={
        "id": 'urn:uuid: 1b', 
        "wkt_geometry": [51.917591, 7.549667, 52.00137, 7.704849], 
        'time': ['1980-01-11T02:28:00Z', '1990-01-11T02:28:00Z'], 
        'vector': [[52.00137, 7.549667], [52.00137, 7.704849], [51.917591, 7.704849], [51.917591, 7.549667]], 
        'raster': False}



muenster1t3 ={
        "id": 'urn:uuid: 1c', 
        "wkt_geometry": [51.917591, 7.549667, 52.00137, 7.704849], 
        'time': None, 
        'vector': [[52.00137, 7.549667], [52.00137, 7.704849], [51.917591, 7.704849], [51.917591, 7.549667]], 
        'raster': False}

muenster1t4 ={
        "id": 'urn:uuid: 1d', 
        "wkt_geometry": [51.917591, 7.549667, 52.00137, 7.704849], 
        'time': None, 
        'vector': [[52.00137, 7.549667], [52.00137, 7.704849], [51.917591, 7.704849], [51.917591, 7.549667]], 
        'raster': False}


#################################
#### different datatype #########
#################################

muenster1d1 ={
        "id": 'urn:uuid: 1dd', 
        "wkt_geometry": [51.917591, 7.549667, 51.917591, 7.549667], 
        'time': ['2000-01-11T02:28:00Z', '2010-01-11T02:28:00Z'], 
        'vector': [[52.00137, 7.549667]], 
        'raster': False}

muenster1d2 ={
        "id": 'urn:uuid: 1dd', 
        "wkt_geometry": [51.917591, 7.549667, 52.00137, 7.704849], 
        'time': ['2000-01-11T02:28:00Z', '2010-01-11T02:28:00Z'], 
        'vector': [[52.00137, 7.549667], [52.00137, 7.704849], [51.917591, 7.704849], [51.917591, 7.549667]],
        'raster': True}


#################################
##### different geo extent ######
#################################



australien ={
        'id': 'urn:uuid:2', 
        "wkt_geometry": [-51.917591, -7.549667, -52.00137, -7.704849], 
        'time': None, 
        'vector': [[-52.00137, -7.549667], [-52.00137, -7.704849], [-51.917591, -7.704849], [-51.917591, -7.549667]], 
        'raster': False}

Muenster2 ={
        'id': 'urn:uuid:3', 
        'wkt_geometry': [51.917168, 7.548981, 52.00137, 7.704849], 
        'time': None, 
        'vector': [[51.931988, 7.548981], [51.99841, 7.549667], [52.00137, 7.554131], [52.001158, 7.699699], [51.99841, 7.704849], [51.921403, 7.704506], [51.917168, 7.693176], [51.917168, 7.598419], [51.917168, 7.554474]], 
        'raster': False
    }
Muenster3 ={
        'id': 'urn:uuid:4', 
        'wkt_geometry': [51.917168, 7.551041, 52.00137, 7.704163], 
        'time': None, 
        'vector': [[51.917168, 7.551041], [52.00137, 7.704163], [51.917168, 7.551041]], 
        'raster': False
    }

Muenster4 ={
        'id': 'urn:uuid:5', 
        'wkt_geometry': [51.898529, 7.413025, 52.054179, 7.816086], 
        'time': ['2007-06-11T02:28:00Z', '2007-08-11T02:28:00Z'], 
        'vector': [[51.945535, 7.413025], [52.054179, 7.732315], [51.994183, 7.816086], [51.898529, 7.488556]], 
        'raster': False
    }

Mitteldeutschland ={
        'id': 'urn:uuid:6', 
        'wkt_geometry': [51.618017, 7.619019, 52.593038, 9.574585], 
        'time': ['2007-06-11T02:28:00Z', '2007-08-11T02:28:00Z'], 
        'vector': [[52.593038, 7.619019], [52.593038, 9.574585], [51.618017, 9.574585], [51.618017, 7.619019]], 
        'raster': False
    }

entriesGeo=[muenster1t1, muenster1t2, Muenster2, Muenster3, Muenster4, australien, Mitteldeutschland]
entriesDat=[muenster1t1, muenster1d1, muenster1d2]
entriesTime=[muenster1t1, muenster1t2, muenster1t3, muenster1t4]
entriesAll=[muenster1t1, muenster1t2, Muenster2, Muenster3, Muenster4, australien, Mitteldeutschland, muenster1t3, muenster1t4, muenster1t2, muenster1d1, muenster1d2]

print(getSimilarRecords(entriesGeo, muenster1t1, 10,0,0,2,1,0,5,True))
'''
        ent is an entry and therefor the same format
        n : number of similar records to be retrieved
        ext : weight of extent similarity 
        dat : weight of datatype similarity 
        loc : weight of location similarity
        geo : weight geographic similarity
        tim : weight temporal similarity
        max : max value for weights
        dtl : boolean, true if detailed
'''

