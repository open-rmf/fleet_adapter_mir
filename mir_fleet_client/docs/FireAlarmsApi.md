# mir_fleet_client.FireAlarmsApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**fire_alarms_get**](FireAlarmsApi.md#fire_alarms_get) | **GET** /fire_alarms | GET /fire_alarms
[**fire_alarms_id_get**](FireAlarmsApi.md#fire_alarms_id_get) | **GET** /fire_alarms/{id} | GET /fire_alarms/{id}
[**fire_alarms_id_put**](FireAlarmsApi.md#fire_alarms_id_put) | **PUT** /fire_alarms/{id} | PUT /fire_alarms/{id}


# **fire_alarms_get**
> [GetFireAlarms] fire_alarms_get()

GET /fire_alarms

Deprecated, use /evacuations endpoint instead : Retrieve list of fire alarms

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import fire_alarms_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_fire_alarms import GetFireAlarms
from pprint import pprint
# Defining the host is optional and defaults to http://localhost
# See configuration.py for a list of all supported configuration parameters.
configuration = mir_fleet_client.Configuration(
    host = "http://localhost"
)

# The client must configure the authentication and authorization parameters
# in accordance with the API server security policy.
# Examples for each auth method are provided below, use the example that
# satisfies your auth use case.

# Configure HTTP basic authorization: Basic
configuration = mir_fleet_client.Configuration(
    username = 'YOUR_USERNAME',
    password = 'YOUR_PASSWORD'
)

# Enter a context with an instance of the API client
with mir_fleet_client.ApiClient(configuration) as api_client:
    # Create an instance of the API class
    api_instance = fire_alarms_api.FireAlarmsApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /fire_alarms
        api_response = api_instance.fire_alarms_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling FireAlarmsApi->fire_alarms_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetFireAlarms]**](GetFireAlarms.md)

### Authorization

[Basic](../README.md#Basic)

### HTTP request headers

 - **Content-Type**: Not defined
 - **Accept**: application/json


### HTTP response details

| Status code | Description | Response headers |
|-------------|-------------|------------------|
**200** | Successfully retrieved the list of elements |  -  |
**400** | Invalid ordering or Invalid filters or Wrong output fields or Invalid limits |  -  |
**404** | Not found |  -  |

[[Back to top]](#) [[Back to API list]](../README.md#documentation-for-api-endpoints) [[Back to Model list]](../README.md#documentation-for-models) [[Back to README]](../README.md)

# **fire_alarms_id_get**
> GetFireAlarm fire_alarms_id_get(id)

GET /fire_alarms/{id}

Deprecated, use /evacuations endpoint instead : Retrieve details about the fire alarm with the specified ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import fire_alarms_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_fire_alarm import GetFireAlarm
from pprint import pprint
# Defining the host is optional and defaults to http://localhost
# See configuration.py for a list of all supported configuration parameters.
configuration = mir_fleet_client.Configuration(
    host = "http://localhost"
)

# The client must configure the authentication and authorization parameters
# in accordance with the API server security policy.
# Examples for each auth method are provided below, use the example that
# satisfies your auth use case.

# Configure HTTP basic authorization: Basic
configuration = mir_fleet_client.Configuration(
    username = 'YOUR_USERNAME',
    password = 'YOUR_PASSWORD'
)

# Enter a context with an instance of the API client
with mir_fleet_client.ApiClient(configuration) as api_client:
    # Create an instance of the API class
    api_instance = fire_alarms_api.FireAlarmsApi(api_client)
    id = 1 # int | The id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /fire_alarms/{id}
        api_response = api_instance.fire_alarms_id_get(id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling FireAlarmsApi->fire_alarms_id_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **id** | **int**| The id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetFireAlarm**](GetFireAlarm.md)

### Authorization

[Basic](../README.md#Basic)

### HTTP request headers

 - **Content-Type**: Not defined
 - **Accept**: application/json


### HTTP response details

| Status code | Description | Response headers |
|-------------|-------------|------------------|
**200** | Successfully retrieve the specified element |  -  |
**400** | Invalid ordering or Invalid filters or Wrong output fields or Invalid limits |  -  |
**404** | Not found |  -  |

[[Back to top]](#) [[Back to API list]](../README.md#documentation-for-api-endpoints) [[Back to Model list]](../README.md#documentation-for-models) [[Back to README]](../README.md)

# **fire_alarms_id_put**
> GetFireAlarm fire_alarms_id_put(id, fire_alarm)

PUT /fire_alarms/{id}

Deprecated, use /evacuations endpoint instead : Modify the values of the fire alarm with the specified ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import fire_alarms_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_fire_alarm import GetFireAlarm
from mir_fleet_client.model.put_fire_alarm import PutFireAlarm
from pprint import pprint
# Defining the host is optional and defaults to http://localhost
# See configuration.py for a list of all supported configuration parameters.
configuration = mir_fleet_client.Configuration(
    host = "http://localhost"
)

# The client must configure the authentication and authorization parameters
# in accordance with the API server security policy.
# Examples for each auth method are provided below, use the example that
# satisfies your auth use case.

# Configure HTTP basic authorization: Basic
configuration = mir_fleet_client.Configuration(
    username = 'YOUR_USERNAME',
    password = 'YOUR_PASSWORD'
)

# Enter a context with an instance of the API client
with mir_fleet_client.ApiClient(configuration) as api_client:
    # Create an instance of the API class
    api_instance = fire_alarms_api.FireAlarmsApi(api_client)
    id = 1 # int | The id to modify
    fire_alarm = PutFireAlarm(
        alarm_on=True,
        note="note_example",
        trigger_time=dateutil_parser('1970-01-01T00:00:00.00Z'),
    ) # PutFireAlarm | The new values of the fire_alarm

    # example passing only required values which don't have defaults set
    try:
        # PUT /fire_alarms/{id}
        api_response = api_instance.fire_alarms_id_put(id, fire_alarm)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling FireAlarmsApi->fire_alarms_id_put: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **id** | **int**| The id to modify |
 **fire_alarm** | [**PutFireAlarm**](PutFireAlarm.md)| The new values of the fire_alarm |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetFireAlarm**](GetFireAlarm.md)

### Authorization

[Basic](../README.md#Basic)

### HTTP request headers

 - **Content-Type**: application/json
 - **Accept**: application/json


### HTTP response details

| Status code | Description | Response headers |
|-------------|-------------|------------------|
**200** | The element has been modified successfully |  -  |
**400** | Invalid filters or Invalid JSON or Argument error or Missing content type application/json on the header or Bad request or No fields |  -  |
**404** | Not found |  -  |

[[Back to top]](#) [[Back to API list]](../README.md#documentation-for-api-endpoints) [[Back to Model list]](../README.md#documentation-for-models) [[Back to README]](../README.md)

