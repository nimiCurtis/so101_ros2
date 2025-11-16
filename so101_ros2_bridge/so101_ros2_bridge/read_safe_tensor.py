#!/usr/bin/env python3
"""
Script to read and inspect a .safetensors file
"""

from safetensors import safe_open
import sys

def read_safetensors(file_path):
    """Read and display contents of a safetensors file"""
    
    print(f"Reading: {file_path}\n")
    
    # Open the safetensors file
    with safe_open(file_path, framework="pt") as f:
        # Get all tensor names
        keys = f.keys()
        
        print(f"Number of tensors: {len(keys)}\n")
        print("Tensors in file:")
        print("-" * 60)
        
        for key in keys:
            tensor = f.get_tensor(key)
            print(f"Name: {key}")
            print(f"  Shape: {tensor.shape}")
            print(f"  Dtype: {tensor.dtype}")
            print(f"  Device: {tensor.device}")
            
            # Show some statistics if it's a numeric tensor
            if tensor.numel() > 0:
                print(f"  Min: {tensor.min().item():.6f}")
                print(f"  Max: {tensor.max().item():.6f}")
                print(f"  Mean: {tensor.mean().item():.6f}")
            print()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python read_safetensors.py <path_to_safetensors_file>")
        sys.exit(1)
    
    file_path = sys.argv[1]
    read_safetensors(file_path)