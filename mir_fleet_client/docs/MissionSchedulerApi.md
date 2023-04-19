# mir_fleet_client.MissionSchedulerApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**mission_scheduler_delete**](MissionSchedulerApi.md#mission_scheduler_delete) | **DELETE** /mission_scheduler | DELETE /mission_scheduler
[**mission_scheduler_get**](MissionSchedulerApi.md#mission_scheduler_get) | **GET** /mission_scheduler | GET /mission_scheduler
[**mission_scheduler_id_delete**](MissionSchedulerApi.md#mission_scheduler_id_delete) | **DELETE** /mission_scheduler/{id} | DELETE /mission_scheduler/{id}
[**mission_scheduler_id_get**](MissionSchedulerApi.md#mission_scheduler_id_get) | **GET** /mission_scheduler/{id} | GET /mission_scheduler/{id}
[**mission_scheduler_id_put**](MissionSchedulerApi.md#mission_scheduler_id_put) | **PUT** /mission_scheduler/{id} | PUT /mission_scheduler/{id}
[**mission_scheduler_post**](MissionSchedulerApi.md#mission_scheduler_post) | **POST** /mission_scheduler | POST /mission_scheduler


# **mission_scheduler_delete**
> mission_scheduler_delete()

DELETE /mission_scheduler

Abort all pending missions

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import mission_scheduler_api
from mir_fleet_client.model.error import Error
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
    api_instance = mission_scheduler_api.MissionSchedulerApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # DELETE /mission_scheduler
        api_instance.mission_scheduler_delete()
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MissionSchedulerApi->mission_scheduler_delete: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

void (empty response body)

### Authorization

[Basic](../README.md#Basic)

### HTTP request headers

 - **Content-Type**: Not defined
 - **Accept**: application/json


### HTTP response details

| Status code | Description | Response headers |
|-------------|-------------|------------------|
**204** | The element has been deleted successfully |  -  |
**400** | Invalid filters or Invalid JSON or Argument error or Missing content type application/json on the header or Bad request or No fields |  -  |
**404** | Not found |  -  |

[[Back to top]](#) [[Back to API list]](../README.md#documentation-for-api-endpoints) [[Back to Model list]](../README.md#documentation-for-models) [[Back to README]](../README.md)

# **mission_scheduler_get**
> [GetMissionSchedulers] mission_scheduler_get()

GET /mission_scheduler

Retrieve the list of missions queued in the mission scheduler

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import mission_scheduler_api
from mir_fleet_client.model.get_mission_schedulers import GetMissionSchedulers
from mir_fleet_client.model.error import Error
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
    api_instance = mission_scheduler_api.MissionSchedulerApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /mission_scheduler
        api_response = api_instance.mission_scheduler_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MissionSchedulerApi->mission_scheduler_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetMissionSchedulers]**](GetMissionSchedulers.md)

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

# **mission_scheduler_id_delete**
> mission_scheduler_id_delete(id)

DELETE /mission_scheduler/{id}

Abort the mission schedule with the specified id

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import mission_scheduler_api
from mir_fleet_client.model.error import Error
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
    api_instance = mission_scheduler_api.MissionSchedulerApi(api_client)
    id = 1 # int | The id to delete

    # example passing only required values which don't have defaults set
    try:
        # DELETE /mission_scheduler/{id}
        api_instance.mission_scheduler_id_delete(id)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MissionSchedulerApi->mission_scheduler_id_delete: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **id** | **int**| The id to delete |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

void (empty response body)

### Authorization

[Basic](../README.md#Basic)

### HTTP request headers

 - **Content-Type**: Not defined
 - **Accept**: application/json


### HTTP response details

| Status code | Description | Response headers |
|-------------|-------------|------------------|
**204** | The element has been deleted successfully |  -  |
**400** | Invalid filters or Invalid JSON or Argument error or Missing content type application/json on the header or Bad request or No fields |  -  |
**404** | Not found |  -  |

[[Back to top]](#) [[Back to API list]](../README.md#documentation-for-api-endpoints) [[Back to Model list]](../README.md#documentation-for-models) [[Back to README]](../README.md)

# **mission_scheduler_id_get**
> GetMissionScheduler mission_scheduler_id_get(id)

GET /mission_scheduler/{id}

Retrieve the details about the mission scheduler with the specified id

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import mission_scheduler_api
from mir_fleet_client.model.get_mission_scheduler import GetMissionScheduler
from mir_fleet_client.model.error import Error
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
    api_instance = mission_scheduler_api.MissionSchedulerApi(api_client)
    id = 1 # int | The id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /mission_scheduler/{id}
        api_response = api_instance.mission_scheduler_id_get(id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MissionSchedulerApi->mission_scheduler_id_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **id** | **int**| The id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetMissionScheduler**](GetMissionScheduler.md)

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

# **mission_scheduler_id_put**
> GetMissionScheduler mission_scheduler_id_put(id, mission_scheduler)

PUT /mission_scheduler/{id}

Modify the values of the mission scheduler with the specified id

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import mission_scheduler_api
from mir_fleet_client.model.get_mission_scheduler import GetMissionScheduler
from mir_fleet_client.model.put_mission_scheduler import PutMissionScheduler
from mir_fleet_client.model.error import Error
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
    api_instance = mission_scheduler_api.MissionSchedulerApi(api_client)
    id = 1 # int | The id to modify
    mission_scheduler = PutMissionScheduler(
        earliest_start_time=dateutil_parser('1970-01-01T00:00:00.00Z'),
        high_priority=True,
        mission_id="mission_id_example",
        priority=1,
        robot_id=1,
    ) # PutMissionScheduler | The new values of the mission_scheduler

    # example passing only required values which don't have defaults set
    try:
        # PUT /mission_scheduler/{id}
        api_response = api_instance.mission_scheduler_id_put(id, mission_scheduler)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MissionSchedulerApi->mission_scheduler_id_put: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **id** | **int**| The id to modify |
 **mission_scheduler** | [**PutMissionScheduler**](PutMissionScheduler.md)| The new values of the mission_scheduler |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetMissionScheduler**](GetMissionScheduler.md)

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

# **mission_scheduler_post**
> GetMissionSchedulers mission_scheduler_post(mission_schedulers)

POST /mission_scheduler

Add a new entry in the mission scheduler

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import mission_scheduler_api
from mir_fleet_client.model.get_mission_schedulers import GetMissionSchedulers
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.post_mission_schedulers import PostMissionSchedulers
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
    api_instance = mission_scheduler_api.MissionSchedulerApi(api_client)
    mission_schedulers = PostMissionSchedulers(
        created_by_id="created_by_id_example",
        description="description_example",
        earliest_start_time=dateutil_parser('1970-01-01T00:00:00.00Z'),
        high_priority=True,
        mission_id="mission_id_example",
        parameters=[
            {},
        ],
        priority=1,
        robot_id=1,
    ) # PostMissionSchedulers | The details of the mission_schedulers

    # example passing only required values which don't have defaults set
    try:
        # POST /mission_scheduler
        api_response = api_instance.mission_scheduler_post(mission_schedulers)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MissionSchedulerApi->mission_scheduler_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **mission_schedulers** | [**PostMissionSchedulers**](PostMissionSchedulers.md)| The details of the mission_schedulers |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetMissionSchedulers**](GetMissionSchedulers.md)

### Authorization

[Basic](../README.md#Basic)

### HTTP request headers

 - **Content-Type**: application/json
 - **Accept**: application/json


### HTTP response details

| Status code | Description | Response headers |
|-------------|-------------|------------------|
**201** | The element has been created successfully |  -  |
**400** | Argument error or Missing content type application/json on the header or Bad request or Invalid JSON |  -  |
**409** | Duplicate entry |  -  |

[[Back to top]](#) [[Back to API list]](../README.md#documentation-for-api-endpoints) [[Back to Model list]](../README.md#documentation-for-models) [[Back to README]](../README.md)

