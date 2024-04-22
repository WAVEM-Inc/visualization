from typing import Any;

def filter_empty_values(data: Any) -> Any:
    if isinstance(data, dict):
        return {
            key: filter_empty_values(value)
            for key, value in data.items()
            if value is not None and filter_empty_values(value) is not None
        };
    elif isinstance(data, list):
        filtered_list = [
            filter_empty_values(item)
            for item in data
            if item is not None and filter_empty_values(item) is not None
        ];
        return [item for item in filtered_list if item];
    elif isinstance(data, str):
        return data if data.strip() != "" else None
    else:
        return data;