# mir_fleet_client.PromptApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**prompt_get**](PromptApi.md#prompt_get) | **GET** /prompt | GET /prompt
[**prompt_guid_get**](PromptApi.md#prompt_guid_get) | **GET** /prompt/{guid} | GET /prompt/{guid}
[**prompt_guid_put**](PromptApi.md#prompt_guid_put) | **PUT** /prompt/{guid} | PUT /prompt/{guid}


# **prompt_get**
> GetPrompt prompt_get()

GET /prompt

Get the list of active user prompts for robots that are active on the fleet.

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import prompt_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_prompt import GetPrompt
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
    api_instance = prompt_api.PromptApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /prompt
        api_response = api_instance.prompt_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PromptApi->prompt_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetPrompt**](GetPrompt.md)

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

# **prompt_guid_get**
> GetPromptAnswer prompt_guid_get(guid)

GET /prompt/{guid}

Get the details of a specific prompt by the GUID.

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import prompt_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_prompt_answer import GetPromptAnswer
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
    api_instance = prompt_api.PromptApi(api_client)
    guid = "guid_example" # str | The guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /prompt/{guid}
        api_response = api_instance.prompt_guid_get(guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PromptApi->prompt_guid_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetPromptAnswer**](GetPromptAnswer.md)

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

# **prompt_guid_put**
> GetPromptAnswer prompt_guid_put(guid, prompt_answer)

PUT /prompt/{guid}

Answer a prompt given by the GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import prompt_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.put_prompt_answer import PutPromptAnswer
from mir_fleet_client.model.get_prompt_answer import GetPromptAnswer
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
    api_instance = prompt_api.PromptApi(api_client)
    guid = "guid_example" # str | The guid to modify
    prompt_answer = PutPromptAnswer(
        answer="answer_example",
    ) # PutPromptAnswer | The new values of the prompt_answer

    # example passing only required values which don't have defaults set
    try:
        # PUT /prompt/{guid}
        api_response = api_instance.prompt_guid_put(guid, prompt_answer)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PromptApi->prompt_guid_put: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to modify |
 **prompt_answer** | [**PutPromptAnswer**](PutPromptAnswer.md)| The new values of the prompt_answer |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetPromptAnswer**](GetPromptAnswer.md)

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

