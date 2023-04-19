# mir_fleet_client.SettingGroupsApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**setting_groups_get**](SettingGroupsApi.md#setting_groups_get) | **GET** /setting_groups | GET /setting_groups
[**setting_groups_id_get**](SettingGroupsApi.md#setting_groups_id_get) | **GET** /setting_groups/{id} | GET /setting_groups/{id}
[**setting_groups_settings_group_id_settings_advanced_get**](SettingGroupsApi.md#setting_groups_settings_group_id_settings_advanced_get) | **GET** /setting_groups/{settings_group_id}/settings/advanced | GET /setting_groups/{settings_group_id}/settings/advanced
[**setting_groups_settings_group_id_settings_get**](SettingGroupsApi.md#setting_groups_settings_group_id_settings_get) | **GET** /setting_groups/{settings_group_id}/settings | GET /setting_groups/{settings_group_id}/settings


# **setting_groups_get**
> [GetSettingGroups] setting_groups_get()

GET /setting_groups

Retrieve a list with the settings groups

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import setting_groups_api
from mir_fleet_client.model.get_setting_groups import GetSettingGroups
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
    api_instance = setting_groups_api.SettingGroupsApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /setting_groups
        api_response = api_instance.setting_groups_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling SettingGroupsApi->setting_groups_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetSettingGroups]**](GetSettingGroups.md)

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

# **setting_groups_id_get**
> GetSettingGroup setting_groups_id_get(id)

GET /setting_groups/{id}

Retrieve the details about the settings group with the specified ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import setting_groups_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_setting_group import GetSettingGroup
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
    api_instance = setting_groups_api.SettingGroupsApi(api_client)
    id = "id_example" # str | The id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /setting_groups/{id}
        api_response = api_instance.setting_groups_id_get(id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling SettingGroupsApi->setting_groups_id_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **id** | **str**| The id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetSettingGroup**](GetSettingGroup.md)

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

# **setting_groups_settings_group_id_settings_advanced_get**
> [GetSettingGroupAdvancedSettings] setting_groups_settings_group_id_settings_advanced_get(settings_group_id)

GET /setting_groups/{settings_group_id}/settings/advanced

Retrieve the list of advanced settings from the settings group with the specified settings group ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import setting_groups_api
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
    api_instance = setting_groups_api.SettingGroupsApi(api_client)
    settings_group_id = 1 # int | The settings_group_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /setting_groups/{settings_group_id}/settings/advanced
        api_response = api_instance.setting_groups_settings_group_id_settings_advanced_get(settings_group_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling SettingGroupsApi->setting_groups_settings_group_id_settings_advanced_get: %s\n" % e)
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

# **setting_groups_settings_group_id_settings_get**
> [GetSettingGroupSettings] setting_groups_settings_group_id_settings_get(settings_group_id)

GET /setting_groups/{settings_group_id}/settings

Retrieve the list of settings from the settings group with the specified settings group ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import setting_groups_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_setting_group_settings import GetSettingGroupSettings
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
    api_instance = setting_groups_api.SettingGroupsApi(api_client)
    settings_group_id = 1 # int | The settings_group_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /setting_groups/{settings_group_id}/settings
        api_response = api_instance.setting_groups_settings_group_id_settings_get(settings_group_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling SettingGroupsApi->setting_groups_settings_group_id_settings_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **settings_group_id** | **int**| The settings_group_id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetSettingGroupSettings]**](GetSettingGroupSettings.md)

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

