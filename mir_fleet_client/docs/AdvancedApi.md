# mir_fleet_client.AdvancedApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**setting_groups_settings_group_id_settings_advanced_get**](AdvancedApi.md#setting_groups_settings_group_id_settings_advanced_get) | **GET** /setting_groups/{settings_group_id}/settings/advanced | GET /setting_groups/{settings_group_id}/settings/advanced
[**settings_advanced_get**](AdvancedApi.md#settings_advanced_get) | **GET** /settings/advanced | GET /settings/advanced
[**settings_advanced_id_get**](AdvancedApi.md#settings_advanced_id_get) | **GET** /settings/advanced/{id} | GET /settings/advanced/{id}
[**settings_advanced_id_put**](AdvancedApi.md#settings_advanced_id_put) | **PUT** /settings/advanced/{id} | PUT /settings/advanced/{id}


# **setting_groups_settings_group_id_settings_advanced_get**
> [GetSettingGroupAdvancedSettings] setting_groups_settings_group_id_settings_advanced_get(settings_group_id)

GET /setting_groups/{settings_group_id}/settings/advanced

Retrieve the list of advanced settings from the settings group with the specified settings group ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import advanced_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_setting_group_advanced_settings import GetSettingGroupAdvancedSettings
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
    api_instance = advanced_api.AdvancedApi(api_client)
    settings_group_id = 1 # int | The settings_group_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /setting_groups/{settings_group_id}/settings/advanced
        api_response = api_instance.setting_groups_settings_group_id_settings_advanced_get(settings_group_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling AdvancedApi->setting_groups_settings_group_id_settings_advanced_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **settings_group_id** | **int**| The settings_group_id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetSettingGroupAdvancedSettings]**](GetSettingGroupAdvancedSettings.md)

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

# **settings_advanced_get**
> [GetSettingsAdvanced] settings_advanced_get()

GET /settings/advanced

Retrieve the list with the advanced settings

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import advanced_api
from mir_fleet_client.model.get_settings_advanced import GetSettingsAdvanced
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
    api_instance = advanced_api.AdvancedApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /settings/advanced
        api_response = api_instance.settings_advanced_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling AdvancedApi->settings_advanced_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetSettingsAdvanced]**](GetSettingsAdvanced.md)

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

# **settings_advanced_id_get**
> GetSettingAdvanced settings_advanced_id_get(id)

GET /settings/advanced/{id}

Retrieve the details of the advanced setting with the specified ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import advanced_api
from mir_fleet_client.model.get_setting_advanced import GetSettingAdvanced
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
    api_instance = advanced_api.AdvancedApi(api_client)
    id = "id_example" # str | The id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /settings/advanced/{id}
        api_response = api_instance.settings_advanced_id_get(id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling AdvancedApi->settings_advanced_id_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **id** | **str**| The id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetSettingAdvanced**](GetSettingAdvanced.md)

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

# **settings_advanced_id_put**
> GetSettingAdvanced settings_advanced_id_put(id, setting_advanced)

PUT /settings/advanced/{id}

Modify the values of the advanced setting with the specified ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import advanced_api
from mir_fleet_client.model.get_setting_advanced import GetSettingAdvanced
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.put_setting_advanced import PutSettingAdvanced
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
    api_instance = advanced_api.AdvancedApi(api_client)
    id = "id_example" # str | The id to modify
    setting_advanced = PutSettingAdvanced(
        value="value_example",
    ) # PutSettingAdvanced | The new values of the setting_advanced

    # example passing only required values which don't have defaults set
    try:
        # PUT /settings/advanced/{id}
        api_response = api_instance.settings_advanced_id_put(id, setting_advanced)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling AdvancedApi->settings_advanced_id_put: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **id** | **str**| The id to modify |
 **setting_advanced** | [**PutSettingAdvanced**](PutSettingAdvanced.md)| The new values of the setting_advanced |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetSettingAdvanced**](GetSettingAdvanced.md)

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

