import requests
import json

'''
GET-Request: Grenzfall des maxreccords (maxrecords=50)
Request: valid JSON/XML
Response: 200

'''
BASE_URL = "http://localhost:8000/?"
params = {"page": 2}
#print (BASE_URL)
response = requests.get(BASE_URL + "service=CSW&version=3.0.0&request=GetSimilarRecords&id=urn:uuid:9a669547-b69b-469f-a11f-2d875366bbdc&elementsetname=summary&maxrecords=50&datatype_weight=1&location_weight=2.5&geographic_weight=2.5&applicationformat=application/xml",params=params)

#print (response)
#print (response.json())
print (json.dumps(response.json(), indent=4))
if response.status_code == 200:
    print(response)
    print(response.json())
    print (json.dumps(response.json(), indent=4))
else:
    print(response)
#resp = response.json()

'''
GET-Request: elementsetname=brief
Request: valid JSON/XML
Response: 500

'''
BASE_URL = "http://localhost:8000/?"
params = {"page": 2}
#print (BASE_URL)
response = requests.get(BASE_URL + "service=CSW&version=3.0.0&request=GetSimilarRecords&id=urn:uuid:9a669547-b69b-469f-a11f-2d875366bbdc&elementsetname=brief&maxrecords=4&datatype_weight=4&location_weight=4&geographic_weight=4&applicationformat=application/xml",params=params)
print(response.url)

if response.status_code == 200:
    print(response)
    print(response.json())
    print (json.dumps(response.json(), indent=4))
else:
    print(response)



'''
GET-Request: datatype_weight=1, location_weight=2,geographic_weight=3
Request: valid JSON/XML
Response: 200

'''
BASE_URL = "http://localhost:8000/?"
params = {"page": 2}
#print (BASE_URL)
response = requests.get(BASE_URL + "service=CSW&version=3.0.0&request=GetSimilarRecords&id=urn:uuid:66ae76b7-54ba-489b-a582-0f0633d96493&elementsetname=summary&maxrecords=49&datatype_weight=1&location_weight=2&geographic_weight=3&applicationformat=application/xml",params=params)

#print (response)
#print (response.json())
print (json.dumps(response.json(), indent=4))

resp = response.json()

'''
GET-Request: Wrong version (version=3.0.1)
Request: Invalid value for version: 3.0.1. Value MUST be 2.0.2 or 3.0.0
Response: 404
'''
BASE_URL = "http://localhost:8000/?"
response = requests.get(BASE_URL + "service=CSW&version=3.0.1&request=GetSimilarRecords&id=urn:uuid:9a669547-b69b-469f-a11f-2d875366bbdc&elementsetname=brief&maxrecords=4&datatype_weight=4&location_weight=d&geographic_weight=4&applicationformat=application/xml")
print (json.dumps(response.json(), indent=4))
resp = response.json()
print (response)

'''
GET-Request: Float im maxreccords (maxrecords=5.5)
Request: Parameter value of 'maxrecords' must be integer
Response: 400
'''
BASE_URL = "http://localhost:8000/?"
response = requests.get(BASE_URL + "service=CSW&version=3.0.0&request=GetSimilarRecords&id=urn:uuid:9a669547-b69b-469f-a11f-2d875366bbdc&elementsetname=summary&maxrecords=5.5&datatype_weight=1&location_weight=2.5&geographic_weight=2.5&applicationformat=application/xml")
print (json.dumps(response.json(), indent=4))
resp = response.json()
print (response)

'''
GET-Request: Value not in range maxrecords (maxrecord=52)
Request: Parameter value must be in the range [1,50]
Response: 400
'''
BASE_URL = "http://localhost:8000/?"
response = requests.get(BASE_URL + "service=CSW&version=3.0.0&request=GetSimilarRecords&id=urn:uuid:9a669547-b69b-469f-a11f-2d875366bbdc&elementsetname=summary&maxrecords=52&datatype_weight=1&location_weight=2.5&geographic_weight=2.5&applicationformat=application/xml")
print (json.dumps(response.json(), indent=4))
resp = response.json()
print (response)

'''
GET-Request: Missing Parameter
Request: Missing id parameter
Response: 400
'''
BASE_URL = "http://localhost:8000/?"
response = requests.get(BASE_URL + "service=CSW&version=3.0.0&request=GetSimilarRecords&elementsetname=summary&maxrecords=3&datatype_weight=0&location_weight=2.5&geographic_weight=2.5&applicationformat=application/xml")
print (json.dumps(response.json(), indent=4))
resp = response.json()
print (response)

'''
GET-Request: Invalid ID Parameter
Request: 
Response: 400
'''
BASE_URL = "http://localhost:8000/?"
response = requests.get(BASE_URL + "service=CSW&version=3.0.0&request=GetSimilarRecords&id=&elementsetname=summary&maxrecords=3&datatype_weight=0&location_weight=2.5&geographic_weight=2.5&applicationformat=application/xml")
print (json.dumps(response.json(), indent=4))
resp = response.json()
print (response)

'''
GET-Request: Value not int or float (geographic_weight=d)
Request: Parameter value of 'location_weight' must be integer or float
Response: 400
'''
BASE_URL = "http://localhost:8000/?"
response = requests.get(BASE_URL + "service=CSW&version=3.0.0&request=GetSimilarRecords&id=urn:uuid:9a669547-b69b-469f-a11f-2d875366bbdc&elementsetname=brief&maxrecords=4&datatype_weight=4&location_weight=d&geographic_weight=4&applicationformat=application/xml")
print (json.dumps(response.json(), indent=4))
resp = response.json()
print (response)