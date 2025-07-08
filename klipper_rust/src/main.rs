use pyo3::prelude::*;
use pyo3::types::PyList;

mod util;

#[pyfunction]
fn get_cpu_info_rs() -> PyResult<String> {
    Ok(util::get_cpu_info())
}

#[pyfunction]
fn get_git_version_rs(py: Python, from_file: bool) -> PyResult<PyObject> {
    match util::get_git_version(from_file) {
        Ok((version, file_status_vec, branch, remote, url)) => {
            let file_status_list = PyList::empty(py);
            for (status, file) in file_status_vec {
                let item = (status, file).to_object(py);
                file_status_list.append(item)?;
            }

            let result_tuple = (version, file_status_list, branch, remote, url);
            Ok(result_tuple.to_object(py))
        }
        Err(e) => Err(PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e)),
    }
}

#[pymodule]
fn klipper_rust(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(get_cpu_info_rs, m)?)?;
    m.add_function(wrap_pyfunction!(get_git_version_rs, m)?)?;
    Ok(())
}

// Add a main function for testing purposes
fn main() {
    println!("CPU Info: {}", util::get_cpu_info());
    match util::get_git_version(true) {
        Ok((version, _, _, _, _)) => println!("Git Version: {}", version),
        Err(e) => println!("Git Version Error: {}", e),
    }
}
