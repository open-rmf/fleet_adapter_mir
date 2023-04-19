# mir_fleet_client.WidgetsApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**dashboards_dashboard_id_widgets_get**](WidgetsApi.md#dashboards_dashboard_id_widgets_get) | **GET** /dashboards/{dashboard_id}/widgets | GET /dashboards/{dashboard_id}/widgets
[**dashboards_dashboard_id_widgets_guid_delete**](WidgetsApi.md#dashboards_dashboard_id_widgets_guid_delete) | **DELETE** /dashboards/{dashboard_id}/widgets/{guid} | DELETE /dashboards/{dashboard_id}/widgets/{guid}
[**dashboards_dashboard_id_widgets_guid_get**](WidgetsApi.md#dashboards_dashboard_id_widgets_guid_get) | **GET** /dashboards/{dashboard_id}/widgets/{guid} | GET /dashboards/{dashboard_id}/widgets/{guid}
[**dashboards_dashboard_id_widgets_guid_put**](WidgetsApi.md#dashboards_dashboard_id_widgets_guid_put) | **PUT** /dashboards/{dashboard_id}/widgets/{guid} | PUT /dashboards/{dashboard_id}/widgets/{guid}
[**dashboards_dashboard_id_widgets_post**](WidgetsApi.md#dashboards_dashboard_id_widgets_post) | **POST** /dashboards/{dashboard_id}/widgets | POST /dashboards/{dashboard_id}/widgets


# **dashboards_dashboard_id_widgets_get**
> [GetDashboardWidgets] dashboards_dashboard_id_widgets_get(dashboard_id)

GET /dashboards/{dashboard_id}/widgets

Retrieve the list of widgets of the dashboard with the specified dashboard ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import widgets_api
from mir_fleet_client.model.get_dashboard_widgets import GetDashboardWidgets
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
    api_instance = widgets_api.WidgetsApi(api_client)
    dashboard_id = "dashboard_id_example" # str | The dashboard_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /dashboards/{dashboard_id}/widgets
        api_response = api_instance.dashboards_dashboard_id_widgets_get(dashboard_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling WidgetsApi->dashboards_dashboard_id_widgets_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **dashboard_id** | **str**| The dashboard_id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetDashboardWidgets]**](GetDashboardWidgets.md)

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

# **dashboards_dashboard_id_widgets_guid_delete**
> dashboards_dashboard_id_widgets_guid_delete(dashboard_id, guid)

DELETE /dashboards/{dashboard_id}/widgets/{guid}

Erase the widget with the specified GUID from the dashboard with the specified dashboard ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import widgets_api
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
    api_instance = widgets_api.WidgetsApi(api_client)
    dashboard_id = "dashboard_id_example" # str | The dashboard_id to delete
    guid = "guid_example" # str | The guid to delete

    # example passing only required values which don't have defaults set
    try:
        # DELETE /dashboards/{dashboard_id}/widgets/{guid}
        api_instance.dashboards_dashboard_id_widgets_guid_delete(dashboard_id, guid)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling WidgetsApi->dashboards_dashboard_id_widgets_guid_delete: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **dashboard_id** | **str**| The dashboard_id to delete |
 **guid** | **str**| The guid to delete |
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

# **dashboards_dashboard_id_widgets_guid_get**
> GetDashboardWidget dashboards_dashboard_id_widgets_guid_get(dashboard_id, guid)

GET /dashboards/{dashboard_id}/widgets/{guid}

Retrieve the details about the widget with the specified GUID in the dashboard with the specified dashboard ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import widgets_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_dashboard_widget import GetDashboardWidget
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
    api_instance = widgets_api.WidgetsApi(api_client)
    dashboard_id = "dashboard_id_example" # str | The dashboard_id to search for
    guid = "guid_example" # str | The guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /dashboards/{dashboard_id}/widgets/{guid}
        api_response = api_instance.dashboards_dashboard_id_widgets_guid_get(dashboard_id, guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling WidgetsApi->dashboards_dashboard_id_widgets_guid_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **dashboard_id** | **str**| The dashboard_id to search for |
 **guid** | **str**| The guid to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetDashboardWidget**](GetDashboardWidget.md)

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

# **dashboards_dashboard_id_widgets_guid_put**
> GetDashboardWidget dashboards_dashboard_id_widgets_guid_put(dashboard_id, guid, dashboard_widget)

PUT /dashboards/{dashboard_id}/widgets/{guid}

Modify the values of the widget with the specified GUID in the dashboard with the specified dashboard ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import widgets_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_dashboard_widget import GetDashboardWidget
from mir_fleet_client.model.put_dashboard_widget import PutDashboardWidget
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
    api_instance = widgets_api.WidgetsApi(api_client)
    dashboard_id = "dashboard_id_example" # str | The dashboard_id to modify
    guid = "guid_example" # str | The guid to modify
    dashboard_widget = PutDashboardWidget(
        settings="settings_example",
    ) # PutDashboardWidget | The new values of the dashboard_widget

    # example passing only required values which don't have defaults set
    try:
        # PUT /dashboards/{dashboard_id}/widgets/{guid}
        api_response = api_instance.dashboards_dashboard_id_widgets_guid_put(dashboard_id, guid, dashboard_widget)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling WidgetsApi->dashboards_dashboard_id_widgets_guid_put: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **dashboard_id** | **str**| The dashboard_id to modify |
 **guid** | **str**| The guid to modify |
 **dashboard_widget** | [**PutDashboardWidget**](PutDashboardWidget.md)| The new values of the dashboard_widget |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetDashboardWidget**](GetDashboardWidget.md)

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

# **dashboards_dashboard_id_widgets_post**
> GetDashboardWidgets dashboards_dashboard_id_widgets_post(dashboard_id, dashboard_widgets)

POST /dashboards/{dashboard_id}/widgets

Add a new widget to the dashboard with the specified dashboard ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import widgets_api
from mir_fleet_client.model.get_dashboard_widgets import GetDashboardWidgets
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.post_dashboard_widgets import PostDashboardWidgets
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
    api_instance = widgets_api.WidgetsApi(api_client)
    dashboard_id = "dashboard_id_example" # str | The dashboard_id to add the new resource to
    dashboard_widgets = PostDashboardWidgets(
        dashboard_id="dashboard_id_example",
        guid="guid_example",
        settings="settings_example",
    ) # PostDashboardWidgets | The details of the dashboard_widgets

    # example passing only required values which don't have defaults set
    try:
        # POST /dashboards/{dashboard_id}/widgets
        api_response = api_instance.dashboards_dashboard_id_widgets_post(dashboard_id, dashboard_widgets)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling WidgetsApi->dashboards_dashboard_id_widgets_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **dashboard_id** | **str**| The dashboard_id to add the new resource to |
 **dashboard_widgets** | [**PostDashboardWidgets**](PostDashboardWidgets.md)| The details of the dashboard_widgets |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetDashboardWidgets**](GetDashboardWidgets.md)

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

