#pragma once

#include "ANSCommon.h"

template <typename ArrayType>
class CANSArray2D // Declaration and definition of a template class should be written in the same file.
{
private:
	// Variables
	int m_nSizeX;
	int m_nSizeY;
	bool m_bInitialized;

public:
	// Variables
// 	ArrayType* data;
	ArrayType** data;

	// Functions
	/**
	 * @brief Create a grid map.
	 * @date 2014/10/23
	 * @param nSizeX
	 * @param nSizeY
	 * @return void
	 */
	inline void create(const int nSizeX, const int nSizeY)
	{
		int i, j;

		// Check size
		if(nSizeX <= 0 || nSizeY <= 0)
		{
			ANS_LOG_ERROR("[ANSArray2D] Size of the array should be larger than 0.", true);
		}

		// Allocate
		if(data)
		{
			for(i = 0; i < nSizeX; i++)
			{
				delete [] data[i];
			}

			delete [] data;

			data = 0;
		}

// 		m_pArray = new ArrayType [nSizeX * nSizeY];

		data = new ArrayType* [nSizeX];

		for(i = 0; i < nSizeX; i++)
		{
			data[i] = new ArrayType [nSizeY];
		}

		m_nSizeX = nSizeX;
		m_nSizeY = nSizeY;

		m_bInitialized = true;

		// Initialize
/*
// 		for(i = 0; i < nSizeX * nSizeY; i++)
		for(i = 0; i < nSizeX; i++)
		{
			for(j = 0; j < nSizeY; j++)
			{
				m_pArray[i][j] = (ArrayType)0;
			}
		}
*/
	};

	/**
	 * @brief Set value.
	 * @date 2014/10/23
	 * @param nx
	 * @param ny
	 * @param value
	 * @return void
	 */
	inline void set(const int nx, const int ny, ArrayType value)
	{
		if(data)
		{
			if(nx >= 0 && nx < m_nSizeX && ny >= 0 && ny < m_nSizeY)
			{
	// 			m_pArray[nx + m_nSizeX * ny] = value;
				data[nx][ny] = value;
			}
			else
			{
				ANS_LOG_ERROR("[ANSArray2D] Access violation", true);
			}
		}
		else
		{
			ANS_LOG_ERROR("[ANSArray2D] The array is not initialized.", true);
		}
	};


	/**
	 * @brief Assign values to all elements.
	 * @date 2014/10/28
	 * @param value
	 * @return void
	 */
	inline void set(ArrayType value)
	{
		if(data)
		{
			for(int i = 0; i < m_nSizeX; i++)
			{
				for(int j = 0; j < m_nSizeY; j++)
				{
					data[i][j] = value;
// 					m_pArray[j * m_nSizeX + i] = value;
				}
			}
		}
		else
		{
			ANS_LOG_ERROR("[ANSArray2D] The pointer is not initialized.", true);
		}
	};


	/**
	 * @brief Get value.
	 * @date 2014/10/23
	 * @param nx
	 * @param ny
	 * @return ArrayType
	 */
	inline ArrayType get(const int nx, const int ny)
	{
		if(data)
		{
// 			return m_pArray[nx + m_nSizeX * ny];
			return data[nx][ny];
		}
		else
		{
			ANS_LOG_ERROR("[ANSArray2D] The pointer is not initialized.", true);
		}

		return data[nx][ny];
	};

	/**
	 * @brief Return array size (x).
	 * @date 2014/10/23
	 * @return int
	 */
	int get_size_x(void)
	{
		return m_nSizeX;
	}

	/**
	 * @brief Return array size (y).
	 * @date 2014/10/23
	 * @return int
	 */
	int get_size_y(void)
	{
		return m_nSizeY;
	}

	/**
	 * @brief Return initialization status.
	 * @date 2014/10/28
	 * @return bool
	 */
	bool is_initialized(void)
	{
		return m_bInitialized;
	}

	/**
	 * @brief "=" operator
	 * @date 2014/10/28
	 * @param arr
	 * @return CANSArray2D<ArrayType>&
	 */
	CANSArray2D<ArrayType>& operator =(CANSArray2D<ArrayType>& arr)
	{
		if(this != &arr)
		{
			if(m_nSizeX != arr.get_size_x() || m_nSizeY != arr.get_size_y())
			{
				create(arr.get_size_x(), arr.get_size_y());
			}

			for(int i = 0; i < m_nSizeX; i++)
			{
				for(int j = 0; j < m_nSizeY; j++)
				{
					data[i][j] = arr.get(i, j);
// 					m_pArray[j * m_nSizeX + i] = arr.get(i, j);
				}
			}
		}

		return *this;
	}

	// Constructor and destructor
	CANSArray2D<ArrayType>(void)
		: data(0)
		, m_nSizeX(0)
		, m_nSizeY(0)
		, m_bInitialized(false)
	{
	};

	CANSArray2D<ArrayType>(const int nSizeX, const int nSizeY)
		: data(0)
		, m_nSizeX(0)
		, m_nSizeY(0)
		, m_bInitialized(false)
	{
		create(nSizeX, nSizeY);
	};

	~CANSArray2D<ArrayType>(void)
	{
		int i;

		// Delete
		if(data)
		{
			for(i = 0; i < m_nSizeX; i++)
			{
				delete [] data[i];
			}

			delete [] data;

			data = 0;
		}
	};
};

